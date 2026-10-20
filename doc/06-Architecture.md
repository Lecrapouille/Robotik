# 6. Architecture

This page explains **how Robotik is built and why**. It is for people who
want to change Robotik itself, or to understand what happens behind the API.
You do not need it to write a plugin.

---

## 1. The principle: Robotik orchestrates

Robotik is not a monolithic simulator. It assembles specialised libraries and
keeps each one in its own domain:

```text
                         Robotik
                            │
       ┌────────────────────┼────────────────────┐
       │                    │                    │
   Robot API            Autonomy             Scenarios
       │             skills · BT · scheduler     │
       │                    │                    │
       └───────────── Backends ──────────────────┘
          │            │             │          │
      Pinocchio      MuJoCo      Compages    Hardware
     kinematics     physics     3D world     (future: SO-101...)
```

| Library | Responsible for | Used through |
|---------|-----------------|--------------|
| [Pinocchio](https://github.com/stack-of-tasks/pinocchio) | Forward kinematics, Jacobians, inverse kinematics | `PinocchioBackend` |
| [MuJoCo](https://github.com/google-deepmind/mujoco) | Dynamics, contacts, integration | `MujocoBackend` |
| [Compages](https://github.com/Lecrapouille/Compages) | ECS world (entities, transforms), 3D rendering | `compages::world::World`, `SceneView` |
| [BlackThorn](https://github.com/Lecrapouille/BlackThorn) | Behavior trees | `Cell::factory()`, `SkillNodes.hpp` |
| OpenCV, AprilTag | Image processing (only in some plugins) | detectors of the plugins |

Robotik provides the layer that combines them. It does not want to become a
second Pinocchio or a second MuJoCo.

### Three layers

Like Compages (GPU, world, renderer), Robotik is cut in layers. Each layer
works without the ones above it, so a project takes the smallest one it
needs (examples in [02-Getting-Started.md](02-Getting-Started.md#7-the-three-layers-in-code)):

```text
Package   files and plugins: Simulation (scenario = cell + verdict), RunReport,
  │       World / Scenario / Experiment YAML, Assertions, Plugin ABI and hosts
Task      a job in C++: CellDescription (data), Cell (step loop, controllers,
  │       truth, perception fidelity), Mission, SkillScheduler, Skills,
  │       WorldModel, Planning (PDDL), Grasp, Learning (RL), Agents
Robot     one robot: Robot, RobotSession, openRobot, joints, sensors,
  │       actuators, resources, backends (MuJoCo, Pinocchio), asset packs
Common    units and poses (from Compages), Result, Signal, Config, Random
```

`make check-layers` (`tests/scripts/check_layers.py`) refuses an include
towards a higher layer, and a call of the truth API (`Cell::pose`,
`Cell::query`...) in `Task/Skills` or `Task/Planning`. A future split stays
possible: `Package` could become `Scenario` (files) and `App` (plugins,
hosts) without touching the layers below.

Contracts between modules are written at the top of each header
(`@par Contract`: what it promises, what it expects, how it reports an
error). Errors from files and users are a `Result`; a mistake in the code
throws. Events are `Signal`s: `SkillScheduler::changed`, `Camera::frames`,
`Simulation::assertionChanged` and `Simulation::ended`; the plugin session
listens to them instead of polling.

---

## 2. The full path of a decision

```text
Scenario (YAML)
   ↓
Behavior tree            what to do next
   ↓
SkillScheduler           may it run now? (resources, priorities, faults)
   ↓
Skills                   how to do it
   ↓
Robot / actuators        joint targets, gripper on/off
   ↓
Backend                  MuJoCo or (later) hardware
   ↓
Sensors                  camera, IMU, lidar, force/torque
   ↓
Perception               detectors
   ↓
WorldModel               the beliefs, read by the skills at the next tick
```

> **Decisions produce commands; the robot moves; sensors observe;
> perception updates beliefs; assertions judge on the ground truth.**

---

## 3. One simulation step

`Simulation::step(dt)` (Package, `src/Robotik/Package/Simulation.cpp`)
steps the cell (`Cell::step`, Task, `src/Robotik/Task/Cell.cpp`), then
judges. In this order:

```text
Simulation::step(dt)
   │
   ├── Cell::step(dt)
   │     ├── FaultInjector::update        faults on/off → resources (un)available
   │     ├── controller tick              behavior tree or plan: requests / cancels skills
   │     ├── SkillScheduler::update       admit, preempt, tick the skills
   │     ├── RobotSession prepare/advance/sense   every robot: actuators, one physics step, sensors
   │     ├── GraspModel::update           what the grippers hold
   │     ├── observe()                    perception → WorldModel, or the truth when
   │     │                                the perception fidelity asks for it
   │     ├── track / syncBodies           ground truth of the bodies, contacts
   │     └── Mission::step                the plugin; Mission::status if no controller
   ├── end of run?                        controller, stop_when, timeout, stop: true
   └── AssertionEngine::update            verdicts, on the ground truth
```

The plugin is called after: `Plugin::update(dt)`, from the host.

---

## 4. The robot: model, state, backend

Three separate notions:

| Notion | What | Where |
|--------|------|-------|
| **Model** | The structure: links, joints, limits. From a URDF | `Robot`, `PinocchioBackend` |
| **State** | What happens now: positions, velocities, efforts | `JointSet` |
| **Backend** | How to read and change the state | `RobotBackend` → `MujocoBackend` (→ `HardwareBackend` later) |

`RobotSession` joins them: the robot, its backend and the world.

```cpp
compages::world::World world;
robotik::AssetResolver const assets = robotik::AssetResolver::standard();
std::filesystem::path const urdf = assets.model("robotik.robots.robot_6axis");

robotik::RobotSession robot(world, urdf);
auto physics = std::make_unique<robotik::MujocoBackend>();
(void)physics->load(urdf);
robot.connect(std::move(physics));

robot.step(Seconds(0.01));   // backend, joint states, kinematics, transforms, sensors
```

In practice, `Simulation` builds all this from the world file.

### Simulated or real: the same code

A skill writes:

```cpp
p_context.robot.joints().moveTo(elbow, Radians(0.5));
```

and never:

```cpp
if (simulation) mujoco_command(...); else serial_command(...);
```

The backend does the translation:

```text
            RobotSession
                 │
           RobotBackend
           /          \
     MujocoBackend   HardwareBackend (future)
          │                │
      qpos / qvel     servo / bus
```

> No physical robot has been controlled with Robotik yet. The architecture is
> ready for it; the hardware backend is not written.

### `JointSet`: structure of arrays

Joint states are stored as parallel arrays (positions[], velocities[],
efforts[], targets[]), not as one small object per joint. This keeps memory
compact and loops fast.

Units are part of the API contract:

| Joint | position | velocity | effort |
|-------|----------|----------|--------|
| revolute | angle (rad) | rad/s | torque (N·m) |
| prismatic | length (m) | m/s | force (N) |

Typed handles (`Revolute`, `Prismatic`) carry these units at compile time.
Conversions to the simulator or the hardware stay in the backends.

### Fast data: what is ECS, what stays a class

ECS (entity, component, system) is a way to store data: an *entity* is only
an id, a *component* is plain data put on it (a pose, a reading), a *system*
is a loop over all the entities that have some components. The data of one
kind sits side by side in memory, so the loop is fast. Compages stores the
world this way; Robotik uses it for what runs at every step.

Rules:

1. **Find by name once, then use a handle.** A name is for people and YAML
   files. Searching it walks a tree and compares strings. Code that runs at
   every step keeps a handle found once: `JointId` for a joint, `LinkRef`
   (`Robot::resolve`) for a link, `ResourceId` for a resource. A handle found
   during an episode is dropped at the reset and found again.
2. **Many small values of one kind: arrays.** `JointSet` keeps one array per
   field, not one object per joint (see above).
3. **Results of the step: components.** A sensor puts its last reading on the
   entity of its link (`ImuReading`, `RangeScan`...). The 3D view and the
   inspectors read the components; they do not ask the sensor.
4. **Rare events and choices of the user: classes.** `Mission`, `Plugin`,
   `PlanningEngine`, `Skill`, `SceneView`, `RobotBackend` keep virtual
   methods. They run a few times per step at most, and a plugin must be able
   to add its own kind.

**Measure before you change the storage.** The sensor pilot shows why. With
64 IMUs and 8 lidars on one arm (`tests/Sensors/BenchSensors.cpp`, physics
off, debug build), a step took 130 µs. The profiler gave: 66 % to find the
link of each sensor by its name at each sample, 8 % to find its kinematic
frame by its name, and almost nothing to the virtual `Sensor::sample` calls.
After rule 1 (`LinkRef`), a step takes 27 µs: 4.7 times faster. Same
readings, same seeds, same scenario reports. A contiguous storage per sensor
type would remove the virtual calls, but they cost under 1 % of the step: it
is not worth the change of API today.

What costs now, in order: `compages::world::World::update` (31 %, the scene
graph of Compages, outside the sensors), the IMU computation (16 %), and the
copy of each reading into its component (15 %). The next candidates are, in
this order: the scene graph update, then the actuators and the skills, then
the grasp systems. Each one starts with a benchmark and a profile.

---

## 5. Actuators, resources, faults

An actuator is not only a function that sends a command: it is also a
**resource** of the same name. Skills reserve resources; the scheduler
arbitrates (see [05-Skills-and-Behavior-Trees.md](05-Skills-and-Behavior-Trees.md)).

A fault (`FaultInjector`) makes a resource unavailable. Running skills that
hold it fail with `ResourceLost`; new requests fail with `Unavailable`. Sensor
readings carry a `valid` flag that is false during the fault. No skill has
special fault code: the fault is a fact, the reaction is a decision of the
behavior tree.

---

## 6. Sensors, perception, WorldModel

A sensor is not its processing:

```text
Camera ──► CameraFrame ──► PerceptionPipeline ──► Detections ──► WorldModel
                             ├── ColorDetector
                             ├── AprilTagDetector
                             └── DepthEstimator
```

The core of Robotik does **not** depend on OpenCV. A camera gives a generic
`robotik::Image`; a plugin may use OpenCV, AprilTag or a neural network in its
own detectors.

Every measurement has a common header (`Measurement`): time stamp, frame,
sequence number, validity. `NoiseModel` applies bias, white noise, resolution
and saturation. `LatencyBuffer` delays the delivery but keeps the capture
time.

---

## 7. Truth, description, belief

| | Access | Used by |
|--|--------|---------|
| **Ground truth** | `Cell::pose`, `Cell::query` | assertions, reports, RL reward |
| **Description** | `Cell::described`, `Cell::waypoints` | the mission, to plan |
| **Belief** | `WorldModel`, through `RobotContext` | skills |

The simulator knows `red_cube = (0.40, 0.20, 0.02)`. The robot may believe
`red_cube = (0.39, 0.21, 0.03), confidence 0.87`. A good scenario tests the
whole chain:

```text
truth ─► simulation ─► camera ─► perception ─► belief ─► skill
```

This prevents cheating by giving the skills the perfect positions. The truth
enters the belief only when the scenario chooses a **perception fidelity**
that asks for it, per object (`perception: { fidelity: truth }`, or
`blind: truth` when no camera has an image), and the reports say so. It is a
mock, as a simpler physics is: a test of the plan does not need a detector
(see [03-Worlds-and-Scenarios.md §7](03-Worlds-and-Scenarios.md#7-truth-description-and-belief)).

Three contracts keep it honest:

1. **One entry.** Skills, PDDL observers and the mission read the
   `WorldModel` only; `make check-layers` refuses a truth call in
   `Task/Skills` and `Task/Planning`.
2. **Source and age.** Each belief says where it comes from
   (`BeliefSource`: prior, sensed, noisy truth, truth) and when it was seen.
   With `perception.max_age`, an older belief is unknown to the planner.
3. **An effect is seen, not assumed.** The plan executor checks the effects
   of each step on the beliefs; with `planner.verify: observed`, only a
   belief seen after the start of the step counts, so a failed grasp is not
   taken for a held object.

---

## 8. Rendering is separate

Compages draws the 3D world. Robotik does not render anything itself:

```text
Robotik ──► SceneView ──► Compages
```

The core runs without a window. This is required for CI, tests, RL,
benchmarks, servers, and thousands of episodes. The graphical simulator is a
**client** of Robotik, not its centre.

---

## 9. Hosts and plugins

```text
             ┌──────────────────┐     ┌──────────────────┐
             │ Robotik-Simulator│     │ Robotik-Headless │    the two hosts
             └────────┬─────────┘     └────────┬─────────┘
                      └──────────┬─────────────┘
                                 │ PluginSession: catalog, packs, dlopen, lifecycle
                                 ▼
                     librobotik_<name>.so          a plugin (C ABI border)
                                 │ bindMission
                                 ▼
                       Simulation + Mission        librobotik-core
```

The border between host and plugin is a **C ABI**
(`include/Robotik/Package/Plugin/PluginABI.h`): no C++ class, container or exception
crosses it. `PluginAPI.hpp` wraps it in C++ for plugin authors. The manifest
`plugin.yaml` is read without loading the library; the library is loaded only
for the chosen scenario. Details in [04-Your-First-Plugin.md](04-Your-First-Plugin.md).

---

## 10. Source layout

```text
include/Robotik/          public headers (one umbrella: Robotik.hpp)
src/Robotik/              implementation, same folders (one library, robotik-core)
simulator/                Robotik-Simulator (ImGui window, panels)
headless/                 Robotik-Headless (CLI, reports)
plugins/<Name>/           one plugin each; plugins/plugin.mk is the shared build recipe
data/packs/<id>/          shared asset packs (robots, tools)
tests/                    unit tests (GoogleTest)
external/                 third-party sources (downloaded)
```

| `include/Robotik/` | Contents |
|--------------------|----------|
| `Common/` | `Math.hpp` (units, `Pose`, `Quaternion`, `Vector3`, from Compages), `Result.hpp`, `Signal.hpp`, `Config.hpp` (editable YAML tree), `Random.hpp` (`Seed`), `File.hpp`, `Version.hpp` |
| `Robot/` | `Robot`, `RobotSession`, `openRobot`, `JointSet`, `LinkRef`, `Devices`, `Resources`, actuators (`Motor`, `JointGroup`, `VacuumGripper`), `TeachPendant` |
| `Robot/Sensors/` | `Sensor`, `Camera`, `Imu`, `RangeScanner`, `ForceTorqueSensor`, `Measurement`, noise models, `Image` |
| `Robot/Backends/` | `RobotBackend` (with `contactList`, `bodyPose` for the truth), `MujocoBackend`, `PinocchioBackend`, `SceneView` |
| `Robot/Assets/` | `AssetPack` (a `pack.yaml`), `AssetResolver` (`robotik.tools.vacuum` → file) |
| `Robot/Components/` | Compages components for robots and objects, queries |
| `Task/` | `CellDescription`, `Cell`, `Mission`, `SkillScheduler`, `RobotContext`, `FaultInjector`, `Metrics`, `Status` |
| `Task/Skills/` | `Skill`, `MotionSkills` (`Home`, `MoveJoint(s)`, `MoveTCP`, `Stop`), `SkillNodes` (BT bridge) |
| `Task/Perception/` | `Detector`, `PerceptionPipeline`, `Detection`, `WorldModel`, `Localization` |
| `Task/Planning/` | `Pddl` (symbols, domain schema), `PlanningDomain` (observers, bindings), `PlanningEngine` (Kairos behind it), `PlanExecutor` (plan → behavior tree, checks, replans), `PlanGraph` |
| `Task/Grasp/` | `GraspModel`: how a gripper holds an object, per physics fidelity (`AbstractGrasp`, `RigidGrasp`); container geometry |
| `Task/Learning/` | RL: `Environment`, `Space`, `SimulationEnvironment`, `EnvironmentPool`, `EpisodeLog` |
| `Task/Agents/` | `Agent`, `Observation`, `Action`: a brain that maps observations to actions |
| `Package/` | `Simulation` (scenario = cell + verdict), `RunReport` |
| `Package/Scenario/` | `World`, `Scenario`, `Experiment`, `Assertions`, `Trajectory`, `PackagePath`, `FileRole` |
| `Package/Plugin/` | `PluginABI.h` (C border), `PluginAPI.hpp` (C++ helper), `PluginCatalog`, `PluginSession`, `PluginManager`, `PluginHost` |

---

## 11. Why this separation matters

You can replace **MuJoCo** by a **hardware backend** without rewriting the
scenario, the tree, the skills, the scheduler, the WorldModel or the
assertions. You can replace an **OpenCV detector** by a **neural detector**
without touching the scenario. This interchangeability is most of the value
of Robotik.
