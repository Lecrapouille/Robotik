# 🧪 The experiment file

[⏫ Reference](README.md)

HelloWorld ships `experiments/hello.experiment.yaml`:

```yaml
experiment:
  id: hello_experiment
  seeds: [1, 2]

  cases:
    - id: wave
      scenario: scenarios/wave.scenario.yaml

    - id: skills
      scenario: scenarios/skills.scenario.yaml

    # Same scenario, the arm in fault for one second, and another strategy:
    # a behavior tree that retries Wave. The assertions do not change.
    - id: skills_arm_fault
      scenario: scenarios/skills.scenario.yaml
      controller: { behavior_tree: scenarios/skills_retry.bt.yaml }
      setup:
        - inject_fault: { resource: arm, at: 1.0, action: disable }
        - inject_fault: { resource: arm, at: 2.0, action: restore }

  evaluation:
    assertions:
      - { condition: (joint_limits_respected robot), phase: always }
    metrics: [ time ]
```

```bash
./build/linux-gcc-debug/bin/Robotik-Headless build/linux-gcc-debug/lib/robotik/plugins/HelloWorld/experiments/hello.experiment.yaml \
    --report hello.json
```

```text
[PASS] wave                     seed=1      controller succeeded at 6.01 s  time=6.01
[PASS] wave                     seed=2      controller succeeded at 6.01 s  time=6.01
[PASS] skills                   seed=1      controller succeeded at 4.67 s  time=4.67
[PASS] skills                   seed=2      controller succeeded at 4.67 s  time=4.67
[PASS] skills_arm_fault         seed=1      controller succeeded at 6.43 s  time=6.43
[PASS] skills_arm_fault         seed=2      controller succeeded at 6.43 s  time=6.43
hello_experiment: 6/6 runs passed
```

| Key | Role |
|-----|------|
| `seeds` | Each case runs with each seed. Without a list, each scenario keeps its own seed |
| `world` | Replaces the world of every scenario |
| `cases[].overrides` | Changes applied to the files **before** they are read (see below) |
| `cases[].controller` | Replaces the strategy of the scenario |
| `cases[].setup` | Faults added to those of the scenario |
| `evaluation.assertions` | Added to the assertions of each scenario |
| `evaluation.metrics` | Values read at the end of each run (same syntax as conditions) |

**Overrides** use dotted paths, starting with `world.` or `scenario.`. In a
list, a segment names the element with this `id` (or this index):

```yaml
    - id: wide_layout
      scenario: scenarios/pick_and_place.scenario.yaml
      overrides:
        world.entities.red_cube.randomize: { x: [-0.05, 0.05], y: [-0.05, 0.05] }
        scenario.timeout: 90
```

A case is a **variant**, not a change of the previous case: each run reads its
files again and rebuilds its world. Several seeds of one case start from the
same world, reset.

The JSON report has, for each run: case, scenario, seed, outcome, time,
perception mode, metrics, and every assertion with its verdict, value, bound
and time.

## 🌱 Seeds and reproducibility

```text
seed 42
   ├── world      layout of the props (randomize)
   ├── faults     random faults
   ├── <sensor>   one stream per sensor, by its name
   └── <scenario logic>  what the scenario logic derives from it (RL policy...)
```

Same scenario, same seed: same run, up to rounding. A noise-free channel draws
no random number, so turning off the noise of one sensor does not change the
others.
