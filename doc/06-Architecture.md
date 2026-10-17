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
| [BlackThorn](https://github.com/Lecrapouille/BlackThorn) | Behavior trees | `Simulation::factory()`, `SkillNodes.hpp` |
| OpenCV, AprilTag | Image processing (only in some plugins) | detectors of the plugins |

Robotik provides the layer that combines them. It does not want to become a
second Pinocchio or a second MuJoCo.

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

`Simulation::step(dt)` (in `src/Robotik/Runtime/Simulation.cpp`) does, in
this order:

```text
Simulation::step(dt)
   │
   ├── FaultInjector::update        faults on/off → resources (un)available
   ├── behavior tree tick           requests / cancels skills
   ├── SkillScheduler::update       admit, preempt, tick the skills
   ├── RobotSession::step           joint control, backend (physics), kinematics, sensors
   ├── observe()                    perception → WorldModel (or the oracle)
   ├── track / syncBodies           ground truth of the bodies, contacts
   ├── Mission::step                the plugin; Mission::status if no tree
   ├── end of run?                  controller, stop_when, timeout, stop: true
   └── AssertionEngine::update      verdicts, on the ground truth
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
| **Ground truth** | `Simulation::pose`, `Simulation::query` | assertions, reports, RL reward |
| **Description** | `Simulation::described`, `Simulation::waypoints` | the mission, to plan |
| **Belief** | `WorldModel`, through `RobotContext` | skills |

The simulator knows `red_cube = (0.40, 0.20, 0.02)`. The robot may believe
`red_cube = (0.39, 0.21, 0.03), confidence 0.87`. A good scenario tests the
whole chain:

```text
truth ─► simulation ─► camera ─► perception ─► belief ─► skill
```

This prevents cheating by giving the skills the perfect positions. The truth
enters the belief only through the oracle, only if the scenario asks for it
(`perception: { oracle: when_blind }`), and the reports say so.

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
(`include/Robotik/Plugin/PluginABI.h`): no C++ class, container or exception
crosses it. `PluginAPI.hpp` wraps it in C++ for plugin authors. The manifest
`plugin.yaml` is read without loading the library; the library is loaded only
for the chosen scenario. Details in [04-Your-First-Plugin.md](04-Your-First-Plugin.md).

---

## 10. Source layout

```text
include/Robotik/          public headers (one umbrella: Robotik.hpp)
src/Robotik/              implementation, same folders
simulator/                Robotik-Simulator (ImGui window, panels)
headless/                 Robotik-Headless (CLI, reports)
plugins/<Name>/           one plugin each; plugins/plugin.mk is the shared build recipe
data/packs/<id>/          shared asset packs (robots, tools)
tests/                    unit tests (GoogleTest)
external/                 third-party sources (downloaded)
```

| `include/Robotik/` | Contents |
|--------------------|----------|
| `Agents/` | `Agent`, `Observation`, `Action`: a brain that maps observations to actions |
| `Assets/` | `Version`, `AssetPack` (a `pack.yaml`), `AssetResolver` (`robotik.tools.vacuum` → file) |
| `Backends/` | `RobotBackend` (with `contactList`, `bodyPose` for the truth), `MujocoBackend`, `PinocchioBackend`, `SceneView` |
| `ECS/` | Compages components for robots and objects, queries |
| `Environment/` | RL: `Environment`, `Space`, `SimulationEnvironment`, `EnvironmentPool`, `EpisodeLog` |
| `Math/` | `Geometry.hpp` (`Pose`, `Quat`, `Vector3`), `Random.hpp` (`Seed`), `Si.hpp` (units) |
| `Perception/` | `Detector`, `PerceptionPipeline`, `Detection`, `WorldModel`, `Localization` |
| `Planning/` | `Pddl` (symbols, domain schema), `PlanningDomain` (observers, bindings), `PlanningEngine` (Kairos behind it), `PlanExecutor` (plan → behavior tree, checks, replans) |
| `Plugin/` | `PluginABI.h` (C border), `PluginAPI.hpp` (C++ helper), `PluginCatalog`, `PluginSession`, `PluginManager`, `PluginHost` |
| `Robot/` | `Robot`, `RobotSession`, `JointSet`, actuators (`Motor`, `JointGroup`, `VacuumGripper`), `TeachPendant` |
| `Runtime/` | `Simulation`, `Mission`, `SkillScheduler`, `ResourceManager`, `FaultInjector`, `Metrics`, `RobotContext`, `RunReport`, `Status` |
| `Scenario/` | `ConfigNode`, `WorldDescription`, `Scenario`, `Experiment`, `AssertionEngine`, `Trajectory` |
| `Scene/` | Container geometry (`inside`) |
| `Sensors/` | `Sensor`, `Camera`, `Imu`, `RangeScanner`, `ForceTorqueSensor`, `Measurement`, `NoiseModel`, `Image` |
| `Skills/` | `Skill`, `MotionSkills` (`Home`, `MoveJoint(s)`, `MoveTCP`, `Stop`), `SkillNodes` (BT bridge) |
| `Systems/` | `GraspModel`: how a gripper holds an object, per physics fidelity (`AbstractGrasp`, `RigidGrasp`) |

---

## 11. Why this separation matters

You can replace **MuJoCo** by a **hardware backend** without rewriting the
scenario, the tree, the skills, the scheduler, the WorldModel or the
assertions. You can replace an **OpenCV detector** by a **neural detector**
without touching the scenario. This interchangeability is most of the value
of Robotik.
