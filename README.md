![RobotIK](doc/logos/RobotIK.png)

# 🤖 RobotIK

**RobotIK is a lightweight C++20 library for building robots that can be simulated, tested, trained, and eventually driven on real hardware—without rewriting their logic.**

RobotIK is not meant to replace excellent existing robotics libraries. It aims to make them work together behind a simple API. RobotIK started as a personal project to deepen my knowledge of robotics and inverse kinematics (IK), then evolved into a lightweight architecture focused on simulation: a way to explore, with hindsight, how I would organize simulation, tests, and control around a clear API if I were building a robot today.

> **Write a mission once, then run it against a simulated robot or a real one.**

The same behavior can start in a simulator, run headless in CI, feed an RL environment, and later connect to physical hardware:

> ⚠️ **Active project: the API is still evolving.** Scenarios, demos, and tests are the best examples of the current API. PDDL planning is available; Prolog, GOAP and LLM front-ends are planned.

```text
                         Your application
                               │
                               ▼
                         ┌───────────┐
                         │  RobotIK  │
                         └─────┬─────┘
                               │
             ┌─────────────────┼─────────────────┐
             │                 │                 │
          Perception         Skills          Scenarios
             │                 │                 │
          WorldModel      Behavior Tree       Tests
             │                 │
             └────────────┬────┘
                          │
                    Robot / Commands
                          │
              ┌───────────┴───────────┐
              │                       │
           MuJoCo                 Real robot
          simulation              SO-101, ...
```

## ❓ Why RobotIK?

Modern robotics already has strong specialized libraries:

- **Pinocchio** for kinematics and dynamics;
- **MuJoCo** for physics and contact simulation;
- **Ruckig**, **OMPL**, etc. for trajectories and motion planning;
- **OpenCV** for image processing;
- **behaviortree.cpp** for behavior trees (a compact alternative to state machines);
- **RViz** for 3D visualization.

The interesting problem is not rewriting all of that, but:

> **How do you build a simple autonomous robot by combining these building blocks?**

RobotIK provides that orchestration layer. It builds on [Pinocchio](https://github.com/stack-of-tasks/pinocchio) (kinematics), [MuJoCo](https://github.com/google-deepmind/mujoco) (physics), [Kairos](https://github.com/PlanSys2/kairos) (PDDL planner), and two other projects of mine:

- 🧱 [Compages](https://github.com/Lecrapouille/Compages) — a C++20 3D library in three layers that work alone: **GPU** (shaders and compute, data set by the names of the shader), **World** (scene graph + ECS, joints, headless), **Renderer** (scene, glTF/URDF assets, pictures). RobotIK keeps its world in Compages and draws it in the simulator. [Documentation](external/Compages/doc/README.md).
- 🌳 [BlackThorn](https://github.com/Lecrapouille/BlackThorn) — a behavior tree library with one YAML file per tree, a real blackboard, simulated time, and **Oakular**, an open-source editor and live visualizer embedded in the RobotIK simulator. [Documentation](external/BlackThorn/doc/README.md).

Some perception demos require OpenCV and AprilTag; they stay in their plugins.

---

## 🛠️ A concrete example

Imagine a 6-DOF arm that must pick a red cube and place it in a box. You want to express:

```text
Detect the cube
→ approach
→ reach the cube
→ grasp
→ lift the cube
→ move to the box
→ release
→ verify the cube is in the box
```

RobotIK breaks this mission into several levels:

```text
Mission
   │
   ▼
Behavior Tree
   │
   ├── Detect(red_cube)
   ├── Approach(red_cube)
   ├── Reach(red_cube)
   ├── Grasp(red_cube)
   ├── MoveTo(box)
   └── Release(red_cube)
          │
          ▼
       Skills
          │
          ▼
     Actuators
          │
          ▼
       Robot
```

A `Skill` does not need to know whether the robot is simulated. In RobotIK, a **`Skill`** is the unit of action between the behavior tree and the robot.

```text
MoveTCP(target)
       │
       ▼
   RobotIK API
       │
       ├───────────────┐
       ▼               ▼
    MuJoCo          SO-101
   simulation        real
```

---

## 🧪 A mission can be a test

A RobotIK scenario file describes not only the world and how the robot should behave, but also **how to tell whether the mission succeeded**—a contract that doubles as an integration test.

For example:

```yaml
assertions:
  - controller.success
  - inside(red_cube, box)
  - holding(gripper) == 0
  - { condition: "collision_count(robot) == 0", phase: always }
  - { condition: "pose_error(easel) < 0.002", phase: always }
  - time < 30
```

A failed assertion keeps the measured value, the expected bound and the simulated time. Zero collision does not prove that nothing moved: measure it.

The same mission can be used as:

- a demo;
- a regression test;
- a benchmark;
- a perception experiment;
- a learning environment;
- a test bed for a new planning strategy.

> **Assertions are judged at every step (`always`), at the end, at a given time, within a deadline, eventually, or as an ordered sequence. The grammar borrows ideas from OpenSCENARIO without depending on it.**

---

## 🏭 Simulation or real robot?

This is a central idea in RobotIK. A skill must not contain:

```cpp
if (mujoco)
{
    ...
}
else if (so101)
{
    ...
}
```

It should drive the robot through a common API:

```cpp
robot.joint("shoulder").moveTo(target);
robot.gripper().open();
```

Behind that API, a `RobotBackend` can be plugged in:

```text
RobotBackend
   │
   ├── MujocoBackend
   ├── SO101Backend       ← future
   └── other robot        ← future
```

The backend translates RobotIK commands to the simulator or hardware. That also enables a **digital twin** in Compages: the real robot publishes state, RobotIK updates the world, and Compages renders it.

> **Note: no physical robot has been controlled with this library yet.**

---

## 🏗️ Architecture (work in progress)

```text
                         RobotIK
                            │
       ┌────────────────────┼────────────────────┐
       │                    │                    │
 Robot / State          Autonomy            Scenarios
       │                    │                    │
       │              ┌─────┼─────┐              │
       │              │     │     │              │
       │             GOAP  PDDL  LLM             │
       │              │     │     │              │
       │              └─────┼─────┘              │
       │                    ▼                    │
       │              Behavior Tree              │
       │                    │                    │
       │                  Skills                 │
       │                    │                    │
       └─────────────── Robot API ───────────────┘
                            │
                            ▼
                 ┌──────────┴──────────┐
                 │                     │
              MuJoCo               Hardware
                 │                     │
              Physics              SO-101...
```

The planning side is intentionally extensible:

- ✅ **Behavior trees** (BlackThorn) turn plans into reactive execution;
- ✅ **PDDL** (Kairos): a scenario gives a goal instead of a tree, the planner finds the actions from the robot's beliefs and replans when a skill fails ([doc/11-PDDL-Planner.md](doc/11-PDDL-Planner.md));
- 🔜 **GOAP** for game-style action planning;
- 🔜 **Prolog** to express and query world logic;
- 🔜 **LLM** to interpret intent or propose high-level plans.

One possible flow:

```text
"Pick the red cube and put it in the box"
                  │
                  ▼
             LLM / user
                  │
                  ▼
           GOAP / PDDL planner
                  │
            symbolic plan
                  │
                  ▼
          Behavior Tree
                  │
                  ▼
                Skills
                  │
                  ▼
        control / motion
```

Prolog can run in parallel to answer questions such as:

```text
inside(red_cube, box).
reachable(arm, red_cube).
holding(gripper, red_cube).
```

None of these technologies is mandatory; they are interchangeable bricks.

---

## 👁️ Perception

A RobotIK camera must not force OpenCV into the core library. The pattern is:

```text
Camera
  │
  ▼
FrameSource
  │
  ▼
PerceptionPipeline
  │
  ├── Detector
  ├── Detector
  └── DepthEstimator
        │
        ▼
    WorldModel
```

For example, the line follower adds its own detectors to the pipeline of the simulation (`plugins/LineFollower/src/LineFollowerMission.cpp`):

```cpp
p_simulation.perception().add<AprilTagDetector>(TAG_SIZE_M);
p_simulation.perception().add<LineDetector>();
```

The core gives `Camera`, `FrameSource`, `PerceptionPipeline`, `Detector` and `DepthEstimator`; detectors that need OpenCV or AprilTag (`ColorDetector`, `AprilTagDetector`, `LineDetector`) live in the plugins. RobotIK core stays independent of rendering and image processing, and a mission still runs headless, without camera images.

---

## 🧩 World, scenario, experiment

RobotIK describes an experiment in three files that do not talk about the same thing:

| File | Says |
|------|------|
| **World** | What physically exists: the robot, its tool, bodies, props, sensors |
| **Scenario** | What happens, when it stops, and what is checked |
| **Experiment** | Which variants to run and compare, with which seeds and metrics |

```yaml
# worlds/pick_place_cell.world.yaml
world:
  id: pick_place_cell
  entities:
    - id: robot
      type: robot
      model: robotik.robots.robot_6axis      # asset id, resolved in data/packs
      sensors:
        wrist_camera: { type: camera, parent: link6, fov: 70, frequency: 15 }
      actuators:
        arm: { type: joint_group }
    - id: vacuum
      type: tool
      model: robotik.tools.vacuum
      attach: { parent: robot.flange, frame: tool_mount }
      actuators:
        gripper: { type: vacuum }
    - id: red_cube
      type: prop
      shape: { type: box, size: [0.04, 0.04, 0.04] }
      pose: { position: [0.40, 0.20, 0.02] }
      randomize: { x: [-0.02, 0.02], y: [-0.02, 0.02] }
    - id: box
      type: prop
      shape: { type: container, size: [0.16, 0.16, 0.08] }
      pose: { position: [0.40, -0.20, 0.04] }
```

```yaml
# scenarios/pick_and_place.scenario.yaml
scenario:
  id: pick_and_place
  world: worlds/pick_place_cell.world.yaml           # paths start at the plugin folder
  seed: 42
  controller:
    behavior_tree: scenarios/pick_and_place.bt.yaml
  timeout: 60
  assertions:
    - controller.success
    - inside(red_cube, box)
    - { condition: "collision_count(robot) == 0", phase: always }
```

```yaml
# experiments/pick_and_place.experiment.yaml
experiment:
  id: pick_and_place_experiment
  seeds: [11, 12, 13]
  cases:
    - { id: nominal, scenario: scenarios/pick_and_place.scenario.yaml }
    - id: wide_layout
      scenario: scenarios/pick_and_place.scenario.yaml
      overrides:
        world.entities.red_cube.randomize: { x: [-0.05, 0.05], y: [-0.05, 0.05] }
  evaluation:
    metrics: [ time, collision_count(robot) ]
```

The scenario does not say how the robot solves the task: the controller is only the default strategy, and the same assertions judge any other one. Details in [doc/03-Worlds-and-Scenarios.md](doc/03-Worlds-and-Scenarios.md).

---

## 🌀 Reproducibility

Robotics experiments are hard to reproduce when randomness is involved:

- initial poses;
- sensor noise;
- faults;
- RL policy;
- environment.

RobotIK uses a master seed:

```yaml
seed: 42
```

It can feed the various random generators:

```text
seed = 42
     │
     ├── world
     ├── sensors
     ├── faults
     └── RL run
```

That helps debugging: the same seed should yield the same experiment under identical conditions.

---

## 🏆 Reinforcement learning

RobotIK provides an `Environment` abstraction and an `EnvironmentPool`. You can run:

```text
Environment 0 ─┐
Environment 1 ─┤
Environment 2 ─┤
Environment 3 ─┼──▶ RL policy
...            │
Environment N ─┘
```

without opening N windows. Rendering is an optional observation path for the user, not a core dependency. That keeps the loop:

```text
observation
    ↓
policy
    ↓
action
    ↓
physics
    ↓
reward
    ↓
observation
```

independent of environment count and thread count. A `SimulationEnvironment` turns any scenario into an environment from three swappable parts (observation builder, action adapter, reward evaluator); spaces are named and bounded, observations read from the ground truth are flagged as privileged, and an episode ends in success, failure, time limit or technical abort. See [doc/08-Reinforcement-Learning.md](doc/08-Reinforcement-Learning.md).

---

## ⚖️ RobotIK and SkiROS2

RobotIK may look close to **SkiROS2**—both use skills, behavior trees, planning, and world knowledge.

The positioning differs.

**SkiROS2** is a robot control platform on ROS 2, with a semantic world model, skills with pre/post-conditions, behavior trees, and task planning. It fits distributed robotic systems in the ROS ecosystem well.

RobotIK targets writing robotic programs in C++ without adopting a full middleware stack. Another emphasis is symmetry:

```text
             same mission
                  │
        ┌─────────┴─────────┐
        ▼                   ▼
     simulation          hardware
        │                   │
      MuJoCo             SO-101
```

---

## 🚀 Clone, build, and run

**Note:** RobotIK builds on Linux with plain `make` (no CMake). It does not build on macOS: the simulator needs a newer OpenGL than the 4.1 of macOS.

```bash
git clone https://github.com/Lecrapouille/RobotIK --recurse-submodules
cd RobotIK

make download-external-libs
make -j"$(nproc --all)"
```

A `build` directory should contain static and shared libraries plus executables:

```bash
./build/Robotik-Simulator
```

New to robotics or to RobotIK? Read [doc/01-Introduction.md](doc/01-Introduction.md), then follow the **HelloWorld** tutorial in [doc/04-Your-First-Plugin.md](doc/04-Your-First-Plugin.md).

The **Demos** menu lists every package built under `build/plugins/`. The same scenario runs without a window, and so does a whole experiment, with a JSON report:

```bash
./build/Robotik-Headless \
    build/plugins/PickAndPlace/scenarios/pick_and_place.scenario.yaml \
    --seed 7
./build/Robotik-Headless \
    build/plugins/Painter/experiments/painter.experiment.yaml \
    --report painting.json
```

Parallel RL demo:

```bash
./build/Robotik-PickAndPlaceRL \
    --policy converged \
    --envs 16 \
    --threads 8 \
    --episodes 32
```

Optional install:

```bash
sudo make install
```

For developers, unit tests:

```bash
make tests -j"$(nproc --all)"
```

---

## 📚 Documentation

| Document | Contents |
|----------|----------|
| [doc/README.md](doc/README.md) | Reading order, beginner → advanced |
| [doc/01-Introduction.md](doc/01-Introduction.md) | Robotics in ten minutes, and what RobotIK does |
| [doc/02-Getting-Started.md](doc/02-Getting-Started.md) | Build, simulator, headless, reports |
| [doc/03-Worlds-and-Scenarios.md](doc/03-Worlds-and-Scenarios.md) | World, scenario and experiment files, assertions, trajectories, ground truth |
| [doc/04-Your-First-Plugin.md](doc/04-Your-First-Plugin.md) | The HelloWorld tutorial, writing a plugin, manifest, asset packs |
| [doc/05-Skills-and-Behavior-Trees.md](doc/05-Skills-and-Behavior-Trees.md) | Skills, resources, `SkillScheduler`, BlackThorn trees, faults |
| [doc/06-Architecture.md](doc/06-Architecture.md) | Layers, data flow, `include/Robotik/` layout |
| [doc/07-API-Reference.md](doc/07-API-Reference.md) | The classes a plugin uses |
| [doc/08-Reinforcement-Learning.md](doc/08-Reinforcement-Learning.md) | Environments, spaces, episode ends, pool, episode log |
| [doc/09-Demos.md](doc/09-Demos.md) | HelloWorld, pick and place, painter, line follower, RL, fly brain, humanoid walk |
| [doc/10-Tools-and-Frames.md](doc/10-Tools-and-Frames.md) | Flange, tool, TCP |
| [doc/11-PDDL-Planner.md](doc/11-PDDL-Planner.md) | The PDDL planner (Kairos): a goal instead of a behavior tree |

The libraries RobotIK is built on have their own documentation, written the same way:

| Library | Documentation | For |
|---------|---------------|-----|
| 🧱 Compages | [external/Compages/doc/README.md](external/Compages/doc/README.md) | the 3D world, the scene, live textures (the Painter canvas) |
| 🌳 BlackThorn | [external/BlackThorn/doc/README.md](external/BlackThorn/doc/README.md) | every node of a `.bt.yaml` tree, the blackboard, the Oakular editor |

🤖 Coding agents: [AGENTS.md](AGENTS.md) maps each task to a skill of `.claude/skills/` (plugin, scenario, skill and behavior tree, simulator, docs). Compages and BlackThorn have their own `AGENTS.md` and skills.

---

## 📝 License

RobotIK is dual-licensed: GNU GPL v3 or later, or a commercial license for those who cannot use the GPL. See [LICENSING.md](LICENSING.md).
