# 🎬 The scenario file

[⏫ Reference](README.md)

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

  perception: { blind: truth }      # true poses when no camera has an image

  faults:
    - { at: 4.0, resource: wrist_camera, action: disable }
    - { at: 6.0, resource: wrist_camera, action: restore }
    - { resource: gripper, rate: 0.01 }        # random: 1 % chance per second

  timeout: 60
  stop_when: (inside red_cube box)

  assertions:
    - controller.success
    - (inside red_cube box)
    - { condition: (== (collision_count robot) 0), phase: always }
```

| Key | Role |
|-----|------|
| `world` | The world file, relative to the package root (see below) |
| `seed` | Master seed. The layout, the sensor noise and the random faults come from it |
| `controller` | The default strategy: a `behavior_tree`, or a goal for the PDDL `planner` ([The PDDL planner: reference](pddl-planner.md)). Without it, the plugin scenario logic drives the robot |
| `trajectories` | Named paths, one file each ([Trajectories](#-trajectories)) |
| `perception` | Where the beliefs come from: `fidelity`, `blind`, `overrides`, `noise`, `max_age` ([truth, description and belief](../concepts/02-worlds-scenarios-experiments.md#-truth-description-and-belief)) |
| `faults` | Planned (`at`) or random (`rate` per second) faults. A fault makes a resource unavailable |
| `physics` | Physics fidelity: `abstract` (default) or `rigid`, per entity (see below) |
| `localization` | Where the base pose of the skills comes from: `truth` (default) or `plugin` ([Localization](#-localization)) |
| `timeout` | Maximum simulated time, in seconds |
| `stop_when` | A condition that ends the run as soon as it is true |
| `assertions` | What is checked ([Assertions](#-assertions)) |

**File names.** Every file named in a world, scenario or experiment file (`world`, `behavior_tree`, `trajectories`, `scenario`, a URDF path) is relative to the **package root**: the folder of `plugin.yaml`. It is the same in every file, whatever the folder of the file:

```text
📦 doc/tutorials/HelloWorld/            ⬅️ package root
┣ 📄 plugin.yaml
┣ 📂 worlds/
┃ ┗ 📄 arm.world.yaml             ⬅️ "worlds/arm.world.yaml"
┣ 📂 scenarios/
┃ ┗ 📄 skills.scenario.yaml       ⬅️ "scenarios/skills.scenario.yaml"
┗ 📂 experiments/
  ┗ 📄 hello.yaml                 ⬅️ names "scenarios/skills.scenario.yaml" too
```

A name may not leave the package. `..` and symbolic links are allowed, but the real file (its canonical path) must be inside the package folder; an absolute path is refused. Otherwise the load fails with the reason. So a package is a closed folder: you can copy or install it as a whole. A model of an asset pack is named by its id (`robotik.tools.vacuum`), not by a path.

## 🔬 Physics fidelity

How finely the physics simulates the objects. The same skills, tree, planner and assertions work at every level.

| Level | Grasp | Release | Use |
|-------|-------|---------|-----|
| `abstract` (default) | the object follows the tool | the object is put on the floor of the container below, at its centre (no fall) | check the logic and the plan, fast |
| `rigid` | the object is a rigid body; a weld holds it to the tool, as it is | the weld goes: the object **falls for real** and stops where it lands | check the task with real positions |
| `contact` | (not available yet) the tool holds by force: it may slip | | study the grasp itself |

```yaml
scenario:
  physics:
    fidelity: abstract        # for every entity
    overrides:                # per entity or actuator
      red_cube: rigid
```

A vacuum cup takes a box by its **top or bottom face**: its tip must be within 2 cm of the centre of that face. A cup pointing up holds a tray from below (DualArm `tray`).

Rules:

- A prop at `rigid`, or every box when a gripper is at `rigid`, becomes a free body of MuJoCo. Its `mass:` matters then (the arm carries it): give a real value, for example `0.05` kg for a 4 cm wooden cube.
- When an object is rigid, the containers are rigid too (static walls), otherwise it would fall through them. A rigid container cannot be randomized.
- A world with rigid props gets a floor even without `floor:`.
- The level does not change during a run.
- Keep the scenario at `abstract` and add the realistic level in an **experiment** case: the same task is then judged at both levels.

  ```yaml
  - id: rigid
    scenario: scenarios/pick_and_place.scenario.yaml
    overrides:
      scenario.physics.fidelity: rigid
  ```

  The report gives the level of each run (`physics` in `--report`): do not compare metrics of two levels.

## 🧭 Localization

A skill reads the base pose of its robot with `robot.base()`. On a real robot nobody knows the true pose of a mobile base: an algorithm estimates it (wheel odometry, landmarks). The simulator knows it. So the robot keeps two bases:

| Base | Read by | Written by |
|------|---------|------------|
| Estimated: `Robot::base()`, `worldPose`, `requireWorldPose` | skills, observers, the scenario logic when it decides | the cell (`truth`) or the plugin (`RobotSession::estimateBase`) |
| True: `RobotSession::trueBase()`, `trueWorldPose`, `requireTrueWorldPose` | sensors, rendering, grasp models, truth queries | the backend (`RobotSession::measureBase`) |

```yaml
localization: truth    # truth (default) | plugin
```

| Level | The estimated base is | Use it to |
|-------|-----------------------|-----------|
| `truth` | the true base, copied at each step: a mock, as `perception: truth` | test the task without a localization algorithm |
| `plugin` | what the plugin writes with `estimateBase`; the start pose until then | test a localization or a calibration algorithm |

A robot bolted to a table has the same base in both levels. The report gives the level (`localization` in `--report`; the console says it when it is not `truth`).

## 🏁 When does a run end?

A run ends when:

1. the controller (behavior tree or scenario logic) **succeeds** or **fails**;
2. `stop_when` becomes true;
3. `timeout` is reached;
4. an assertion declared with `stop: true` fails.

`Simulation::outcome()` gives the reason. After the end, the physics may go on (the simulator keeps the image alive), but the verdicts are frozen.

## ✅ Assertions

An assertion is a **condition**, judged at a **moment**.

```yaml
assertions:
  - controller.success                                    # short form: judged at the end
  - { condition: (== (collision_count robot) 0), phase: always }
  - { condition: (< (pose_error easel) 0.002), phase: always }
  - { condition: (on canvas easel), phase: end }
  - { condition: (< (distance robot.tcp red_cube) 0.05), within: 5.0 }
  - { condition: (holding gripper), eventually: true }
  - { condition: (> time 1), at: 2.0 }
  - name: brush loaded before the first stroke
    sequence: [ (skill_succeeded dip), (skill_started mark) ]
  - { condition: (joint_limits_respected robot), phase: always, stop: true }
```

### ⏱ Moments

| Moment | Verdict |
|--------|---------|
| `phase: end` (default) | Judged at the end. During the run, the simulator shows its current value |
| `phase: always` | Must be true at every step. The first violation is kept (value, time) |
| `at: t` | Judged once, at time `t`. Fails if the run ends before |
| `within: t` | Must become true before time `t` |
| `eventually: true` | Must become true before the end |
| `sequence: [...]` | The conditions must become true in this order |

Two keys change how a step is judged:

| Key | Effect |
|-----|--------|
| `for: t` | With `phase: always`: the condition may stay false up to `t` seconds in a row. A longer run of false steps fails, dated at its start |
| `unknown: skip` | A step where the condition has no answer is not judged (default `fail`). A condition that never has an answer still fails at the end |

### 🧠 Check what the robot believes

The other assertions judge the world. These ones judge the robot: does it know what it does? A robot can be wrong and still look fine for a while. A cup that took the blue cube instead of the red one still holds a cube, and the robot labels it `red_cube`. The world shows the mistake only at the end, when the wrong cube is in the box. Compare the belief with the truth to see it when it happens:

```yaml
- { condition: (= (believed (holding red_cube)) (holding gripper red_cube)),
    phase: always, for: 0.1, unknown: skip }
- { condition: (< (belief_error red_cube) 0.01),
    phase: always, for: 0.5, unknown: skip }
- { condition: (< (base_error robot) 0.05), phase: always, for: 1.0 }
```

A belief is often late, and that is not a fault. Choose `for:` from the latency of what feeds the belief:

- The label of a cup comes one step after the contact (the grasp skill reads the vacuum at its next tick).
- A camera updates a belief only at its rate and after its latency.
- A carried object has no belief: `belief_error` has no answer while it is held.
- An observer answers "unknown" when its beliefs are too old: use `unknown: skip`, or the assertion fails at that step.

A failed assertion keeps the **measured value**, the **expected bound** and the **simulated time**. It does **not** stop the run, unless `stop: true`.

### ❓ Conditions

A condition uses the notation of PDDL: the operator comes first, inside parentheses. You read `(< time 30)` as "time is less than 30". The planner goals use the same notation (`(in red_cube box)`, see [the PDDL planner](../concepts/05-pddl-planner.md)), so you learn one way to write a fact.

| You write | It is true when |
|-----------|-----------------|
| `controller.success` | the value is not zero |
| `(< time 30)` | the comparison holds; the operators are `< <= > >= = == !=` |
| `(>= laps (collision_count robot))` | a comparison of two values |
| `(and c1 c2 ...)` | every condition holds |
| `(or c1 c2 ...)` | one condition holds |
| `(not c)` | `c` does not hold |

A value is a number, a metric, or a query on the world:

| Value | Measures |
|-------|----------|
| `time` | Simulated time (s) |
| `controller.success` | 1 if the controller succeeded |
| plugin metrics | Any name published by the plugin: `hello.ticks`, `laps`, `cross_track.max`… |
| `(pose_error e)` | How far `e` moved since the start of the episode (m) |
| `(orientation_error e)` | How much `e` turned since the start (rad) |
| `(distance a b)` | Distance between two entities or frames, like `robot.tcp` or `robot.tool0` (m) |
| `(inside a b)` | `a` rests in the container `b`, or is inside the box `b` |
| `(on a b)` | `a` is placed on `b`: a contact under `a`, or touching faces for two props |
| `(touching a b)` | `a` and `b` are in contact |
| `(collision_count e)` | Peak number of simultaneous contacts of `e` with other entities, floor excluded. For the robot, the mounted tool and a carried object are included. The contacts of a tool with the entities of its `touches` are not counted ([Tools](world-file.md#-tools)) |
| `(tool_is_attached t robot.flange)` | 1 if the tool `t` is mounted there |
| `(joint_limits_respected robot)` | 1 if every joint is inside its limits (± 1e-3) |
| `(holding g)`, `(holding g object)` | The gripper `g` holds something, or this object |
| `(skill_started s)`, `(skill_succeeded s)`, `(skill_failed s)` | History of the skill scheduler |
| `(believed (p a b))` | What the robot **believes** of the PDDL literal: 1 or 0 from the observer of `p` on the beliefs; no answer when it cannot tell. Needs a planner (the observers) |
| `(belief_error e)` | Distance between where the robot believes `e` is and where it is (m). No answer when `e` was never seen, its belief is older than `perception.max_age`, or a gripper carries it |
| `(base_error robot)` | Distance between the estimated and the true base ([Localization](#-localization)) (m) |

A plugin can add its own queries with `RoboticCell::addQuery` in `ScenarioLogic::setup`. A `;` starts a comment up to the end of the line. Put a name with spaces between double quotes.

RobotIK reads every condition when the scenario loads. A wrong condition stops the load with exit code 1, and the message names the column:

```text
pick_and_place.scenario.yaml: assertion condition '(< tme 30)': column 4: unknown value 'tme' (did you mean 'time'?)
pick_and_place.scenario.yaml: assertion condition '(inside red_cub box)': column 1: unknown entity 'red_cub' (did you mean 'red_cube'?)
```

The same check refuses a wrong number of arguments and an unknown query. A value that has no answer during the run makes the assertion fail with the reason. Example: `(distance robot.tcp cube)` without a tool gives `Unknown Pinocchio frame: tcp`; use `robot.tool0`.

> **Zero collision does not prove that nothing moved.** The easel can slide under a badly modelled contact. This is why the Painter also checks `(pose_error easel)`. Measure what you care about.

## 📈 Trajectories

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

In C++, `RoboticCell::requireWaypoints("red")` returns a cartesian trajectory in the robot base frame. Its frame is placed where the **world file** describes it: if the easel slides during the run, the robot does not know.

## 💻 From C++

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

`robotik::ScenarioSource` keeps the files as an editable tree (`ConfigNode`): `source.override("world.entities.easel.friction", ConfigNode("0.3"))`, then `Scenario::read(source, assets)`. This is what an experiment does.

Next: write the C++ part, in [Your first plugin](../tutorials/04-your-first-plugin.md).
