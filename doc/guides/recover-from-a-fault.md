# 💥 Inject a fault and recover from it

[⏫ Guides](README.md)

A real robot loses a camera, a gripper or an arm during a job. This guide
breaks a device of the HelloWorld skills scenario on purpose, then makes the
behavior tree react to it. Faults are listed in
[the scenario file](../reference/scenario-file.md); the resources and the
scheduler are explained in
[Skills, the scheduler and behavior trees](../concepts/skills-and-behavior-trees.md).

## 💥 Inject a fault

In `plugins/HelloWorld/scenarios/skills.scenario.yaml`, add a fault on the arm, just
before `timeout`:

```yaml
  faults:
    - { at: 1.0, resource: arm, action: disable }
```

```bash
make robotik_hello_world
./build/linux-gcc-debug/bin/Robotik-Headless build/linux-gcc-debug/lib/robotik/plugins/HelloWorld/scenarios/skills.scenario.yaml --trace
```

```text
  0 s  Home  SUCCEEDED  (0 s)
  0.01 s  Wave  FAILED (resource lost: arm)  (0.99 s)
[FAIL] controller.success  (controller.success = 0)
```

The skill `Wave` was running; at 1 s it lost the arm and failed. No code was
written to handle this fault: the scheduler did it. How the robot *reacts* is
the job of the behavior tree: see
[Recover from a fault](#-recover-from-a-fault).

## 🩹 Recover from a fault

Start from HelloWorld lesson 4. In `plugins/HelloWorld/scenarios/skills.scenario.yaml`,
add a fault that takes the arm away between 1 s and 2 s:

```yaml
  faults:
    - { at: 1.0, resource: arm, action: disable }
    - { at: 2.0, resource: arm, action: restore }
```

**Step 1: no recovery.** Run it:

```bash
make robotik_hello_world
./build/linux-gcc-debug/bin/Robotik-Headless build/linux-gcc-debug/lib/robotik/plugins/HelloWorld/scenarios/skills.scenario.yaml --trace
```

```text
  0 s  Home  SUCCEEDED  (0 s)
  0.01 s  Wave  FAILED (resource lost: arm)  (0.99 s)
[FAIL] controller.success  (controller.success = 0)
```

**Step 2: retry.** Change the tree to retry `Wave` until it works. This is
exactly `scenarios/skills_retry.bt.yaml`:

```yaml
      - UntilSuccess:
          attempts: 1000
          child:
            - Action:
                name: Wave
```

```text
  0 s  Home  SUCCEEDED  (0 s)
  0.01 s  Wave  FAILED (resource lost: arm)  (0.99 s)
  1.02 s  Wave  FAILED (unavailable: arm)  (0 s)
  ...
  1.98 s  Wave  FAILED (unavailable: arm)  (0 s)
  2 s  Wave  SUCCEEDED  (3.77 s)
  5.78 s  Home  SUCCEEDED  (0.63 s)
[PASS] controller.success  (controller.success = 1)
```

Look at what you did **not** write: no `if (arm_broken)` in the skill, no
special code in the mission. The **fault** is a fact (the scheduler sees
it); the **reaction** is a decision (the tree makes it). Two separate jobs.

The HelloWorld experiment compares both trees on the same faulty scenario:
see [The experiment file](../reference/experiment-file.md).
