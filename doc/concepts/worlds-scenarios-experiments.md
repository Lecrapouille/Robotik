# 🌍 Worlds, scenarios and experiments

[⏫ Concepts](README.md)

You can do a lot with Robotik **without writing C++**: place objects, add
sensors, inject faults, decide what "success" means, compare variants. All of
this is YAML. This page explains why a run is described by three files, and
what each part of RobotIK knows about the world. The keys of each file are in
the reference: [the world file](../reference/world-file.md),
[the scenario file](../reference/scenario-file.md) and
[the experiment file](../reference/experiment-file.md). The tutorial
[A cube in a box](../tutorials/03-a-cube-in-a-box.md) writes your first one.

## ❓ Three files, three questions

A Robotik experiment answers one question:

> **"In this world, when this happens, does the robot succeed? And how do we
> know?"**

The answer is split into three files. Each one talks about one thing only.

| File | Question | Example |
|------|----------|---------|
| **World** (`worlds/*.world.yaml`) | *What exists, and where?* | the arm, its tool, a cube, a box |
| **Scenario** (`scenarios/*.scenario.yaml`) | *What happens, when does it stop, what do we check?* | "put the cube in the box, in less than 30 s, without collision" |
| **Experiment** (`experiments/*.experiment.yaml`) | *Which variants do we run and compare?* | 3 layouts × 3 seeds |

```text
experiment ──uses──► scenario ──uses──► world
 (variants)          (task + checks)    (things)
```

None of these files says **how** the robot solves the task. The scenario
gives a default strategy (a behavior tree, or the plugin C++ code); an
experiment can replace it. The **same assertions** judge every strategy.

The files live in a plugin folder:

```text
plugins/HelloWorld/
├── plugin.yaml                     lists the scenarios and experiments
├── worlds/arm.world.yaml
├── scenarios/
│   ├── hello.scenario.yaml  wave.scenario.yaml  reach.scenario.yaml  skills.scenario.yaml
│   ├── skills.bt.yaml               a behavior tree
│   └── skills_retry.bt.yaml
└── experiments/hello.experiment.yaml
```

> **Important.** A scenario or an experiment must be listed in `plugin.yaml`.
> Otherwise Headless answers `scenario is not in a plugin package`.

> **Important.** RobotIK refuses a key it does not know, in every file
> (world, scenario, experiment, trajectory, `plugin.yaml`). A typo stops the
> load with exit code 1 and names the closest key:
> `scenario: unknown key 'timout' (did you mean 'timeout'?)`. Free values
> for your scenario logic go under `scenario.parameters`, which is not checked.
> In a flow map (`{ ... }`), put a text with a comma between quotes:
> `name: "Pick and place (PA-BT, faults)"`.

### 🏷 File names: the double extension

The name of a file says its role: `<name>.<role>.yaml`. You see what a file
is without opening it, and tools (the simulator Files panel, the catalog)
use it.

| Double extension | Role | Example |
|------------------|------|---------|
| `.world.yaml` | world | `worlds/arm.world.yaml` |
| `.scenario.yaml` | scenario | `scenarios/wave.scenario.yaml` |
| `.rl.yaml` | scenario driven by a learned policy (reinforcement learning) | `scenarios/pick_and_place.rl.yaml` |
| `.bt.yaml` | behavior tree | `scenarios/skills.bt.yaml` |
| `.domain.pddl` | PDDL domain of the planner ([The PDDL planner](pddl-planner.md)) | `scenarios/pick_and_place.domain.pddl` |
| `.trajectory.yaml` | trajectory ([Trajectories](../reference/scenario-file.md#-trajectories)) | `scenarios/trajectories/red_stroke.trajectory.yaml` |
| `.experiment.yaml` | experiment | `experiments/hello.experiment.yaml` |

Rules:

- Use `.yaml`, not `.yml`.
- The file name gives the `id`: drop `.yaml`, drop `.scenario` (the default
  role), and change every other `.` into `_`. Write this id in the file and
  in `plugin.yaml`:

  | File | id |
  |------|----|
  | `wave.scenario.yaml` | `wave` |
  | `pick_and_place.rl.yaml` | `pick_and_place_rl` |
  | `hello.experiment.yaml` | `hello_experiment` |

  Without `entries:` in `plugin.yaml`, the catalog reads the id this way.
- `plugin.yaml` and `pack.yaml` keep their fixed names.
- It is a convention: the loaders read the content and do not refuse a file
  for its name. An older `wave.yml` still loads.

## 🔍 Truth, description and belief

Three things must not be mixed:

| | What it is | Who can read it |
|--|------------|-----------------|
| **Ground truth** | The real state of the simulated world: `RoboticCell::requirePose`, `RoboticCell::requireQuery` | Assertions, reports, the RL reward |
| **Description** | The world as the file describes it at the start: `RoboticCell::requireDescribedPose`, `RoboticCell::requireWaypoints` | The scenario logic, to plan (like a map given to a real robot) |
| **Belief** | What the robot thinks: the `WorldModel`, filled by perception | The skills, through `RobotContext` |

Skills see only the `RobotContext`: **they cannot cheat**. The ground truth
goes into the belief only when the scenario asks for it, with a
**perception fidelity**, as the physics has its own fidelity:

```yaml
perception:
  fidelity: sensed          # sensed (default) | noisy_truth | truth
  blind: truth              # used instead of sensed when no camera has an image
  overrides:                # per object
    red_cube: truth
  noise: { sigma: 0.005, dropout: 0.1, range: 2.0 }   # for noisy_truth
  max_age: 2.0              # older beliefs are unknown to the planner (s)
```

| Level | Where the belief comes from | Use it to |
|-------|-----------------------------|-----------|
| `sensed` | The detectors, on the camera images | Test the whole chain, perception included |
| `noisy_truth` | The true pose, with gaussian noise (`sigma`, m), misses (`dropout`, a chance per step) and a range (`range`, m) | Test the robustness of a plan without writing a detector |
| `truth` | The true pose, copied | Skip perception: test the plan or the motion only |

`truth` is a **mock**: like a car simulator that gives the lane marks and
the pedestrians instead of a photo-realistic image to process. It is a
choice, made by the scenario and written in the report: each run says the
level of each object (`perception: truth, red_cube: sensed`). The level of
an object is its `overrides` entry, or `fidelity`; when it is `sensed` and
no camera has an image source (headless, no rendering), `blind` is used.

The old key `oracle` is still read: `always` is `fidelity: truth`,
`when_blind` is `blind: truth`, `never` is the default.

**Frame of the beliefs.** With one robot, the beliefs are in its base frame.
With several robots, they share the beliefs in the **world frame**: each
camera turns its detections into the world with the base pose of its robot,
and a skill brings a belief into its own base with
`context.robot.base().pose.inverse() * belief` (`RoboticCell::beliefFrame()`
says which frame is used).

**Judge the beliefs too.** The assertions read the truth, so they see a
mistake of the robot only by its effect, often at the end. To see a wrong
belief when it happens (a cup labelled with the wrong cube, a lost
localization), compare the belief with the truth: `(believed ...)`,
`(belief_error e)`, `(base_error robot)`, with a tolerance for the latency
of the belief ([Check what the robot believes](../reference/scenario-file.md#-check-what-the-robot-believes)).
The base a skill reads is a belief as well: `localization: truth` makes it
the true base, a mock that the report states
([Localization](../reference/scenario-file.md#-localization)).
