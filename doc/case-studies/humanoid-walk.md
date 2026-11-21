# Humanoid walk

[⏫ Case studies](README.md)

A LimX HU_D04_01 humanoid (31 joints) walks in a circle with the walking
policy trained by LimX. No training here: the policy (`walk.onnx`) and the
robot come from the LimX repositories (Apache-2.0), and ONNX Runtime runs the
policy. It shows that a **learned controller** is just another skill.

```bash
make PLUGINS+=HumanoidWalk             # fetches its repositories and ONNX Runtime, prepares its data once
./build/linux-gcc-debug/bin/Robotik-Headless build/linux-gcc-debug/lib/robotik/plugins/HumanoidWalk/scenarios/walk_circle.scenario.yaml --trace
```

```text
  0 s  Stand  SUCCEEDED  (0.999 s)
  1 s  Walk  SUCCEEDED  (24.999 s)
[PASS] controller.success  (controller.success = 1)
[PASS] (> humanoid.height_min 0.7)  (humanoid.height_min = 0.882378)
[PASS] (> humanoid.turns 1)  (humanoid.turns = 1.30405)
[PASS] (> humanoid.radius 0.6)  (humanoid.radius = 0.918705)
[PASS] (< humanoid.radius 1.4)  (humanoid.radius = 0.918705)
```

| Scenario | What happens |
|----------|--------------|
| `walk` | (simulator) the robot walks until you stop the run; the **Humanoid** panel changes the command (forward, left, turn, Stop) |
| `walk_circle` | (test) stand 1 s, then walk 25 s on a circle of `vx / yaw_rate` = 1 m, judged by assertions |

How it works:

- The tree is two skills, `Stand` then `Walk` (`walk_circle.bt.yaml`).
- `WalkMission` plugs a **floating-base** MuJoCo backend, without gravity
  compensation, with the PD gains of the policy on each joint.
- The plugin owns the clock (`clock: plugin` in `plugin.yaml`): it steps the
  physics every 1 ms, and `Walk` runs the policy every 10 steps (100 Hz) on
  the **IMU and the joint encoders only**. Ground truth only feeds the
  metrics (`humanoid.height_min`, `humanoid.turns`, `humanoid.radius`, …).
- When the IMU is lost (fault) or the robot lies down, `Walk` fails and puts
  the joints in damping mode, as the real robot does: it falls.

More details in [plugins/HumanoidWalk/README.md](../../plugins/HumanoidWalk/README.md).
