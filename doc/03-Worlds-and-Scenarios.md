# 3. Worlds, scenarios and experiments

You can do a lot with Robotik **without writing C++**: place objects, add
sensors, inject faults, decide what "success" means, compare variants. All of
this is YAML. This page teaches it with small tutorials, then gives the full
reference.

---

## 1. Three files, three questions

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

### File names: the double extension

The name of a file says its role: `<name>.<role>.yaml`. You see what a file
is without opening it, and tools (the simulator Files panel, the catalog)
use it.

| Double extension | Role | Example |
|------------------|------|---------|
| `.world.yaml` | world | `worlds/arm.world.yaml` |
| `.scenario.yaml` | scenario | `scenarios/wave.scenario.yaml` |
| `.rl.yaml` | scenario driven by a learned policy (reinforcement learning) | `scenarios/pick_and_place.rl.yaml` |
| `.bt.yaml` | behavior tree | `scenarios/skills.bt.yaml` |
| `.domain.pddl` | PDDL domain of the planner ([chapter 11](11-PDDL-Planner.md)) | `scenarios/pick_and_place.domain.pddl` |
| `.trajectory.yaml` | trajectory (§8) | `scenarios/trajectories/red_stroke.trajectory.yaml` |
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

---

## 2. Tutorial: put a cube in a box

You will add two objects to the HelloWorld world and check a fact about them.
Work in the sources, then run `make` to copy them to `build/`.

**Step 1.** Open `plugins/HelloWorld/worlds/arm.world.yaml`. It has one entity, the
robot. Add a red cube and a box at the end of the `entities` list:

```yaml
    - id: red_cube
      type: prop
      shape: { type: box, size: [0.04, 0.04, 0.04] }    # 4 cm cube
      color: [0.9, 0.1, 0.1]                             # red, green, blue in [0, 1]
      pose: { position: [0.40, 0.20, 0.02] }

    - id: box
      type: prop
      shape: { type: container, size: [0.16, 0.16, 0.08] }   # an open box
      pose: { position: [0.40, -0.20, 0.04] }
```

A `prop` is an object that does not move by itself (no physics), unless the
robot grasps it. Positions are in metres, z up. The position is the **centre**
of the object: the cube is 4 cm high, so its centre is at z = 0.02.

**Step 2.** Open `plugins/HelloWorld/scenarios/hello.scenario.yaml` and add two
assertions:

```yaml
  assertions:
    - controller.success
    - hello.ticks > 100
    - time < 3
    - distance(red_cube, box) < 0.05
    - inside(red_cube, box)
```

**Step 3.** Build and run:

```bash
make -C plugins/HelloWorld
./build/Robotik-Headless build/plugins/HelloWorld/scenarios/hello.scenario.yaml
```

```text
[FAIL] distance(red_cube, box) < 0.05  (distance(red_cube, box) = 0.4005)
[FAIL] inside(red_cube, box)  (inside(red_cube, box) = 0)
```

Of course: the cube is 40 cm away from the box.

**Step 4.** "Cheat": put the cube in the box directly. The box floor is 5 mm
thick, so the cube centre is at 0.005 + 0.02 = 0.025:

```yaml
      pose: { position: [0.40, -0.20, 0.025] }
```

```text
[PASS] distance(red_cube, box) < 0.05  (distance(red_cube, box) = 0.015)
[PASS] inside(red_cube, box)  (inside(red_cube, box) = 1)
hello: 1/1 runs passed
```

**Step 5.** Open it in the simulator to see the scene:

```bash
./build/Robotik-Simulator build/plugins/HelloWorld/scenarios/hello.scenario.yaml
```

Of course, a real mission would ask the robot to *move* the cube. This is
what `plugins/PickAndPlace` does, with the same world entities and the same
`inside(red_cube, box)` assertion. When you are done, put the files back with
`git checkout plugins/HelloWorld`.

---

## 3. Tutorial: randomness and seeds

Real objects are never exactly where we expect. Ask Robotik to move the cube
a little at each run:

```yaml
    - id: red_cube
      type: prop
      shape: { type: box, size: [0.04, 0.04, 0.04] }
      pose: { position: [0.40, 0.20, 0.02] }
      randomize: { x: [-0.02, 0.02], y: [-0.02, 0.02] }   # ± 2 cm
```

The random numbers come from the **seed** of the scenario (`seed: 1`). Same
seed, same run, always. Another seed, another layout:

```bash
./build/Robotik-Headless build/plugins/HelloWorld/scenarios/hello.scenario.yaml --seed 7
```

The robot knows only the *nominal* position (the one written in the file).
The *true* position, used by the assertions, is the shifted one. This is how
you test that a robot uses its camera instead of the file.

A failed run in CI prints its seed: run it again with `--seed` to see exactly
the same failure.

---

## 4. The world file

### Entities

The world is a list of **entities**. Each has an `id`, a `type`, a 3D model
or a primitive shape, and a pose.

```yaml
world:
  id: painting_lab
  floor: { friction: 0.8 }             # a physical floor for free bodies

  entities:
    - id: robot
      type: robot
      model: robotik.robots.robot_6axis
      home: { joint1: 0.0, joint2: 0.3, joint3: 1.3, joint4: 0.0, joint5: 1.54, joint6: 0.0 }
      actuators:
        arm: { type: joint_group }

    - id: brush
      type: tool
      model: robotik.tools.default
      attach: { parent: robot.flange, frame: tool_mount }

    - id: easel
      type: body
      model: robotik.painter.easel
      physics: free
      friction: 0.6
      pose: { position: [0.65, 0.0, 0.0], rpy: [0.0, 0.0, 3.141592653589793] }

    - id: canvas
      type: body
      shape: { type: box, size: [0.02, 0.40, 0.50] }
      mass: 0.5
      physics: free
      attach:
        parent: easel.canvas_mount
        transform: { position: [0.011, 0.0, 0.251] }
```

| Type | What it is | Physics |
|------|------------|---------|
| `robot` | A robot you control (one or more, see [Several robots](#several-robots)). Keys: `home`, `sensors`, `actuators` | Its chain, fixed at its `pose` |
| `tool` | A tool. It keeps its identity once mounted and brings its own actuators and sensors | Hangs on the flange |
| `body` | A simulated rigid body: a URDF model (`physics: free` or `fixed`) or a box (`free`, with `mass`) | Yes: gravity, contacts, friction |
| `prop` | A kinematic object: it does not move, unless the robot grasps it. Shape `box` or `container` (open box) | No |

Use a `prop` when you want a simple, exact scene. Use a `body` when physics
matters: "does the easel slide when the brush pushes it?".

Any entity may also have a `kind`: its type for the PDDL planner
(`kind: cube`). Only entities with a `kind` are objects of the planning
problem ([11. The PDDL planner](11-PDDL-Planner.md#3-the-four-pieces)).

### Models are asset ids

`model: robotik.robots.robot_6axis` is not a path. It is `<pack>.<model>`: the
file is found in an **asset pack** (`data/packs/robotik.robots/pack.yaml`).
See [04-Your-First-Plugin.md §8](04-Your-First-Plugin.md#8-asset-packs-sharing-3d-models).

### Attach an entity to another

`attach.parent` names an entity, or one of its links: `easel.canvas_mount` is
the link `canvas_mount` of the easel URDF. `transform` places the entity in
that frame. For a free body, this is only the start pose: physics decides
after that.

The URDF describes a *model*. The world describes an *instance*: where it is,
and on what it depends.

### Tools

A tool is always mounted by its link `tool_mount` on the robot flange
(`robot.flange`). Only one tool is mounted on a robot at a time. Tools declared without
`attach` are **in storage**: not built, but the simulator **Teach Pendant** panel can
mount one instead. Details in [10-Tools-and-Frames.md](10-Tools-and-Frames.md).

```yaml
    - id: vacuum
      type: tool
      model: robotik.tools.vacuum
      attach: { parent: robot.flange, frame: tool_mount }
      actuators:
        gripper: { type: vacuum }
    - { id: drill, type: tool, model: robotik.tools.drill }    # in storage
```

### Several robots

A world may have several entities of type `robot`: two arms of a cell, an
arm and a mobile base. They share **one** physics (MuJoCo), **one** resource
manager, **one** skill scheduler and **the beliefs**. From the DualArm tray
cell (`plugins/DualArm/worlds/tray_cell.world.yaml`):

```yaml
    - id: left
      type: robot
      kind: arm
      model: robotik.robots.robot_6axis
      ...
      actuators:
        left_arm: { type: joint_group }
    - id: left_cup
      type: tool
      model: robotik.tools.vacuum
      attach: { parent: left.flange, frame: tool_mount }
      actuators:
        left_gripper: { type: vacuum }

    - id: right
      type: robot
      kind: arm
      model: robotik.robots.robot_6axis
      pose: { position: [0.80, 0.0, 0.0], rpy: [0.0, 0.0, 3.141592653589793] }
      ...
```

Rules:

- Each robot has its own tool: `attach: { parent: <robot>.flange, frame: tool_mount }`.
- The names of the sensors and actuators are **unique in the world**
  (`left_arm`, `right_arm`): they are the names of the resources, and the
  robots share the resource manager. A second `gripper` is refused when the
  world is loaded.
- Each robot keeps the names of its own URDF: both arms have a `joint1`.
- The first robot of the file is `Simulation::robot()`. The others are
  `Simulation::robot("right")`; all of them are `Simulation::robots()`.
- The skills of the core are named after the robot: `left.Home`,
  `right.Stop`. With one robot they stay `Home` and `Stop`.
- The queries take the robot id: `pose(right.tcp)`, `collision_count(left)`,
  `joint_limits_respected(right)`. A contact with the other robot is a
  collision.
- The shared beliefs are in the **world frame** (with one robot, they stay
  in its base frame: see [§7](#7-truth-description-and-belief)).

How a skill drives one robot of the cell:
[05-Skills-and-Behavior-Trees.md](05-Skills-and-Behavior-Trees.md#several-robots).

### Sensors and actuators

Each sensor and each actuator is also a **resource** with the same name:
skills reserve it, faults make it unavailable.

```yaml
      sensors:
        wrist_camera: { type: camera, parent: link6, position: [0.05, 0.0, 0.0],
                        fov: 70, resolution: [320, 240], frequency: 15, noise: 0.01 }
        imu:
          type: imu
          parent: base_link
          frequency: 200
          latency: 0.002
          accelerometer: { sigma: 0.02, bias: 0.05, saturation: [-160, 160] }
          gyroscope: 0.001
        lidar: { type: range, parent: base_link, beams: 180, fov: 360, max_range: 10,
                 noise: { sigma: 0.01, resolution: 0.001 } }
        wrist_ft: { type: force_torque, link: link6, force: 0.5, torque: 0.02 }
      actuators:
        arm: { type: joint_group }                        # no joints listed: all of them
        left_wheel: { type: motor, joint: left_wheel_joint }
        gripper: { type: vacuum }
```

| Sensor | Measures | Frame of the data |
|--------|----------|-------------------|
| `camera`, `rgbd` | an image (and depth) | optical: z forward, x right, y down |
| `imu` | specific force (+9.81 up at rest), angular velocity, attitude | the sensor |
| `range` | one distance per beam; `max_range` when nothing is hit | XY plane of the sensor |
| `force_torque` | force and torque received by the link from its parent | the measured link |

| Actuator | Commands |
|----------|----------|
| `joint_group` | a set of joints (all of them by default) |
| `motor` | one joint |
| `vacuum` | a vacuum gripper: grasps what touches it |

**Noise.** A noise channel is a standard deviation (`noise: 0.01`) or a full
model `{ sigma, bias, resolution, saturation: [min, max] }`, applied in this
order. `latency` delays the delivery of a measure; the measure keeps its
capture time. Every reading has a common header: time stamp, frame, sequence
number, validity (false while the resource is in fault).

---

## 5. The scenario file

A complete example (the repository file does not use every key):

```yaml
scenario:
  id: pick_and_place
  description: Pick the red cube and put it in the box.
  world: worlds/pick_place_cell.world.yaml          # relative to the package root
  seed: 42

  controller:
    behavior_tree: scenarios/pick_and_place.bt.yaml   # the default strategy

  trajectories:
    drop: scenarios/trajectories/drop.trajectory.yaml

  perception: { oracle: when_blind }

  faults:
    - { at: 4.0, resource: wrist_camera, action: disable }
    - { at: 6.0, resource: wrist_camera, action: restore }
    - { resource: gripper, rate: 0.01 }        # random: 1 % chance per second

  timeout: 60
  stop_when: inside(red_cube, box)

  assertions:
    - controller.success
    - inside(red_cube, box)
    - { condition: "collision_count(robot) == 0", phase: always }
```

| Key | Role |
|-----|------|
| `world` | The world file, relative to the package root (see below) |
| `seed` | Master seed. The layout, the sensor noise and the random faults come from it |
| `controller` | The default strategy: a `behavior_tree`, or a goal for the PDDL `planner` ([chapter 11](11-PDDL-Planner.md#5-reference)). Without it, the plugin mission drives the robot |
| `trajectories` | Named paths, one file each (§8) |
| `perception.oracle` | When the ground truth may replace perception (§7) |
| `faults` | Planned (`at`) or random (`rate` per second) faults. A fault makes a resource unavailable |
| `physics` | Physics fidelity: `abstract` (default) or `rigid`, per entity (see below) |
| `timeout` | Maximum simulated time, in seconds |
| `stop_when` | A condition that ends the run as soon as it is true |
| `assertions` | What is checked (§6) |

**File names.** Every file named in a world, scenario or experiment file
(`world`, `behavior_tree`, `trajectories`, `scenario`, a URDF path) is
relative to the **package root**: the folder of `plugin.yaml`. It is the same
in every file, whatever the folder of the file:

```text
plugins/HelloWorld/            <- package root
├── plugin.yaml
├── worlds/arm.world.yaml            "worlds/arm.world.yaml"
├── scenarios/skills.scenario.yaml       "scenarios/skills.scenario.yaml"
└── experiments/hello.yaml     names "scenarios/skills.scenario.yaml" too
```

A name may not leave the package. `..` and symbolic links are allowed, but
the real file (its canonical path) must be inside the package folder; an
absolute path is refused. Otherwise the load fails with the reason. So a
package is a closed folder: you can copy or install it as a whole. A model of an asset pack is named by its id
(`robotik.tools.vacuum`), not by a path.

### Physics fidelity

How finely the physics simulates the objects. The same skills, tree,
planner and assertions work at every level.

| Level | Grasp | Release | Use |
|-------|-------|---------|-----|
| `abstract` (default) | the object follows the tool | the object is put on the floor of the container below, at its centre (no fall) | check the logic and the plan, fast |
| `rigid` | the object is a rigid body; a weld holds it to the tool | the weld goes: the object **falls for real** and stops where it lands | check the task with real positions |
| `contact` | (not available yet) the tool holds by force: it may slip | | study the grasp itself |

```yaml
scenario:
  physics:
    fidelity: abstract        # for every entity
    overrides:                # per entity or actuator
      red_cube: rigid
```

Rules:

- A prop at `rigid`, or every box when a gripper is at `rigid`, becomes a
  free body of MuJoCo. Its `mass:` matters then (the arm carries it): give a
  real value, for example `0.05` kg for a 4 cm wooden cube.
- When an object is rigid, the containers are rigid too (static walls),
  otherwise it would fall through them. A rigid container cannot be
  randomized.
- A world with rigid props gets a floor even without `floor:`.
- The level does not change during a run.
- Keep the scenario at `abstract` and add the realistic level in an
  **experiment** case: the same task is then judged at both levels.

  ```yaml
  - id: rigid
    scenario: scenarios/pick_and_place.scenario.yaml
    overrides:
      scenario.physics.fidelity: rigid
  ```

  The report gives the level of each run (`physics` in `--report`): do not
  compare metrics of two levels.

### When does a run end?

A run ends when:

1. the controller (behavior tree or mission) **succeeds** or **fails**;
2. `stop_when` becomes true;
3. `timeout` is reached;
4. an assertion declared with `stop: true` fails.

`Simulation::outcome()` gives the reason. After the end, the physics may go
on (the simulator keeps the image alive), but the verdicts are frozen.

### Tutorial: inject a fault

In `plugins/HelloWorld/scenarios/skills.scenario.yaml`, add a fault on the arm, just
before `timeout`:

```yaml
  faults:
    - { at: 1.0, resource: arm, action: disable }
```

```bash
make -C plugins/HelloWorld
./build/Robotik-Headless build/plugins/HelloWorld/scenarios/skills.scenario.yaml --trace
```

```text
  0 s  Home  SUCCEEDED  (0 s)
  0.01 s  Wave  FAILED (resource lost: arm)  (0.99 s)
[FAIL] controller.success  (controller.success = 0)
```

The skill `Wave` was running; at 1 s it lost the arm and failed. No code was
written to handle this fault: the scheduler did it. How the robot *reacts* is
the job of the behavior tree: see
[05-Skills-and-Behavior-Trees.md §6](05-Skills-and-Behavior-Trees.md#6-tutorial-recover-from-a-fault).

---

## 6. Assertions

An assertion is a **condition**, judged at a **moment**.

```yaml
assertions:
  - controller.success                                         # short form: judged at the end
  - { condition: "collision_count(robot) == 0", phase: always }
  - { condition: "pose_error(easel) < 0.002", phase: always }
  - { condition: "on(canvas, easel)", phase: end }
  - { condition: "distance(robot.tcp, red_cube) < 0.05", within: 5.0 }
  - { condition: "holding(gripper)", eventually: true }
  - { condition: "time > 1", at: 2.0 }
  - name: brush loaded before the first stroke
    sequence: [ "skill_succeeded(dip)", "skill_started(mark)" ]
  - { condition: "joint_limits_respected(robot)", phase: always, stop: true }
```

### Moments

| Moment | Verdict |
|--------|---------|
| `phase: end` (default) | Judged at the end. During the run, the simulator shows its current value |
| `phase: always` | Must be true at every step. The first violation is kept (value, time) |
| `at: t` | Judged once, at time `t`. Fails if the run ends before |
| `within: t` | Must become true before time `t` |
| `eventually: true` | Must become true before the end |
| `sequence: [...]` | The conditions must become true in this order |

A failed assertion keeps the **measured value**, the **expected bound** and
the **simulated time**. It does **not** stop the run, unless `stop: true`.

### Conditions

`<value> <op> <number>`, with `op` in `== != < <= > >=`. Or a value alone:
true when it is not zero.

| Value | Measures |
|-------|----------|
| `time` | Simulated time (s) |
| `controller.success` | 1 if the controller succeeded |
| mission metrics | Any name published by the plugin: `hello.ticks`, `laps`, `cross_track.max`… |
| `pose_error(e)` | How far `e` moved since the start of the episode (m) |
| `orientation_error(e)` | How much `e` turned since the start (rad) |
| `distance(a, b)` | Distance between two entities or frames, like `robot.tcp` or `robot.tool0` (m) |
| `inside(a, b)` | `a` rests in the container `b`, or is inside the box `b` |
| `on(a, b)` | `a` is placed on `b`: a contact under `a`, or touching faces for two props |
| `touching(a, b)` | `a` and `b` are in contact |
| `collision_count(e)` | Peak number of simultaneous contacts of `e` with other entities, floor excluded. For the robot, the mounted tool and a carried object are included |
| `tool_is_attached(t, robot.flange)` | 1 if the tool `t` is mounted there |
| `joint_limits_respected(robot)` | 1 if every joint is inside its limits (± 1e-3) |
| `holding(g)`, `holding(g, object)` | The gripper `g` holds something, or this object |
| `skill_started(s)`, `skill_succeeded(s)`, `skill_failed(s)` | History of the skill scheduler |

An unknown value, or an unknown entity, makes the assertion fail with the
reason. Example: `distance(robot.tcp, cube)` without a tool gives
`Unknown Pinocchio frame: tcp`; use `robot.tool0`.

> **Zero collision does not prove that nothing moved.** The easel can slide
> under a badly modelled contact. This is why the Painter also checks
> `pose_error(easel)`. Measure what you care about.

---

## 7. Truth, description and belief

Three things must not be mixed:

| | What it is | Who can read it |
|--|------------|-----------------|
| **Ground truth** | The real state of the simulated world: `Simulation::pose`, `Simulation::query` | Assertions, reports, the RL reward |
| **Description** | The world as the file describes it at the start: `Simulation::described`, `Simulation::waypoints` | The mission, to plan (like a map given to a real robot) |
| **Belief** | What the robot thinks: the `WorldModel`, filled by perception | The skills, through `RobotContext` |

Skills see only the `RobotContext`: **they cannot cheat**. The ground truth
goes into the belief only through the **oracle**, and only if the scenario
asks for it:

```yaml
perception: { oracle: when_blind }    # never (default) | when_blind | always
```

`when_blind` uses the oracle when no camera has an image source, that is, in
headless mode (no rendering). The simulator renders the image and uses real
perception. Reports mark the runs made with the oracle (`oracle perception`).

**Frame of the beliefs.** With one robot, the beliefs are in its base frame.
With several robots, they share the beliefs in the **world frame**: each
camera turns its detections into the world with the base pose of its robot,
and a skill brings a belief into its own base with
`context.robot.base().pose.inverse() * belief` (`Simulation::beliefFrame()`
says which frame is used).

---

## 8. Trajectories

A trajectory is a path, in its own file. The scenario gives it a name.

```yaml
# scenarios/trajectories/red_stroke.trajectory.yaml
trajectory:
  type: cartesian
  frame: easel.canvas_mount
  orientation: { rpy: [1.5707963267948966, 0.0, -1.5707963267948966] }
  points:
    - [0.18, 0.12, 0.16]
    - [0.08, 0.08, 0.16]
    - { position: [0.06, 0.08, 0.16], rpy: [1.5707963267948966, 0.0, -1.5707963267948966] }
```

| Type | Points |
|------|--------|
| `cartesian` | Poses of the tool working point, in `frame` (an entity, or `entity.link`; the world if empty) |
| `joint` | Joint configurations, for the joints listed in `joints` |

In C++, `Simulation::waypoints("red")` returns a cartesian trajectory in the
robot base frame. Its frame is placed where the **world file** describes it:
if the easel slides during the run, the robot does not know.

---

## 9. The experiment file

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
      - { condition: "joint_limits_respected(robot)", phase: always }
    metrics: [ time ]
```

```bash
./build/Robotik-Headless build/plugins/HelloWorld/experiments/hello.experiment.yaml \
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

---

## 10. Seeds and reproducibility

```text
seed 42
   ├── world      layout of the props (randomize)
   ├── faults     random faults
   ├── <sensor>   one stream per sensor, by its name
   └── <mission>  what the mission derives from it (RL policy...)
```

Same scenario, same seed: same run, up to rounding. A noise-free channel draws
no random number, so turning off the noise of one sensor does not change the
others.

---

## 11. From C++

A scenario is also a C++ object. This is what Headless does:

```cpp
robotik::AssetResolver const assets = robotik::AssetResolver::standard();
compages::world::World world;
robotik::Simulation simulation(world, robotik::Scenario::load(path, assets));
simulation.reset(robotik::Seed{ 7 });
while (!simulation.finished())
{
    simulation.step(Seconds(0.01));
}
for (robotik::Check const& check : simulation.checks())
{
    if (check.verdict == robotik::Check::Verdict::Failed)
    {
        std::cerr << check.text << ": " << check.detail << '\n';
    }
}
```

`robotik::ScenarioSource` keeps the files as an editable tree (`ConfigNode`):
`source.override("world.entities.easel.friction", ConfigNode("0.3"))`, then
`Scenario::read(source, assets)`. This is what an experiment does.

---

Next: write the C++ part, in [04-Your-First-Plugin.md](04-Your-First-Plugin.md).
