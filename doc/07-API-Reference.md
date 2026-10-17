# 7. API reference

The classes a plugin author uses, grouped by task. For each one: the header,
what it is for, and its main methods. The headers have more details in their
comments; this page tells you where to look.

Conventions in the code:

- namespace `robotik`; units from `Robotik/Math/Si.hpp` (`Seconds`,
  `Radians`, …) are also visible without the namespace;
- parameters start with `p_`, members with `m_`;
- SI units everywhere: metres, radians, seconds, newtons;
- z up, like URDF.

| I want to… | Use | Section |
|------------|-----|---------|
| Write the entry point of my plugin | `Plugin`, `Host`, `Canvas` | [§1](#1-plugin-api) |
| Add behaviour to a simulation | `Mission` | [§2](#2-mission) |
| Read the world, the time, the truth | `Simulation` | [§3](#3-simulation) |
| Move joints, read poses, compute IK | `Robot`, `JointSet` | [§4](#4-robot-and-joints) |
| Use the gripper, a motor | actuators | [§5](#5-actuators) |
| Write a skill | `Skill`, `SkillDescription`, `RobotContext` | [§6](#6-skills) |
| Run, cancel, inspect skills | `SkillScheduler` | [§7](#7-skillscheduler) |
| Reserve parts of the robot, inject faults | `ResourceManager`, `FaultInjector` | [§8](#8-resources-and-faults) |
| Process images, update beliefs | `Detector`, `PerceptionPipeline`, `WorldModel` | [§9](#9-perception-and-beliefs) |
| Publish numbers for assertions | `Metrics` | [§10](#10-metrics) |
| Find a 3D model | `AssetResolver` | [§11](#11-assets) |
| Make an RL environment | `Environment`, `SimulationEnvironment` | [§12](#12-learning) |

One umbrella header includes everything: `#include "Robotik/Robotik.hpp"`.

---

## 1. Plugin API

Header: `Robotik/Plugin/PluginAPI.hpp` (C++ helper over the C border
`Robotik/Plugin/PluginABI.h`).

### `robotik::Plugin`

Base class of your plugin. Override only what you need; every method returns
`ROBOTIK_PLUGIN_OK` by default.

| Method | Called |
|--------|--------|
| `static PluginInfo describe()` | Before the instance exists. Return `{ .id = "..." }`, the same id as `plugin.yaml` |
| `RobotikPluginStatus setup(Host&, std::string_view scenario)` | Before the simulation exists. Create the mission, `bindMission`, menus, panels |
| `start(Host&)` | The run starts |
| `update(Host&, double dt)` | After each step (or instead of it with `host.clock: plugin`) |
| `onEvent(Host&, RobotikEvent const&)` | A subscribed event |
| `pause(Host&, bool paused)` | Pause / Play |
| `stop(Host&)` | End of the run; the simulation still exists |
| `shutdown(Host&)` | Free the mission; the simulation is destroyed |

Then, once, in one `.cpp` file: `ROBOTIK_EXPORT_PLUGIN(MyPlugin)`.

Return codes: `ROBOTIK_PLUGIN_OK`, `ROBOTIK_PLUGIN_ERR_SETUP` (unknown
scenario, bad configuration), `ROBOTIK_PLUGIN_ERR_RUNTIME` (also used when
your method throws: the exception is caught and reported).

### `robotik::Host`

| Method | Does |
|--------|------|
| `bindMission(Mission&)` | The mission the simulation will use. Call it in `setup` |
| `Simulation* simulation()` | The running simulation, or null |
| `log(level, text)`, `error(text)` | Messages. Levels `ROBOTIK_LOG_INFO / _WARNING / _ERROR` |
| `subscribe(RobotikEventKind)` | Receive this kind of event |
| `addMenuItem(menu, item, void(*)(void*), void* user)` | Simulator menu entry; returns a handle |
| `addPanel(id, title, void(*)(void*, RobotikCanvas const*), void* user)` | Simulator panel; returns a handle |
| `removeUi(handle)` | Remove a menu entry or a panel |
| `scenarioPath()` | Path of the scenario file |

`Host` is a small value (a pointer); you may keep a copy in your plugin.

### `RobotikEvent`

| `kind` | Fields |
|--------|--------|
| `ROBOTIK_EVENT_KEY_PRESSED` | `code`: the key (`'H'`) |
| `ROBOTIK_EVENT_SCENARIO_STARTED`, `_STOPPED` | — |
| `ROBOTIK_EVENT_SKILL_COMPLETED`, `_SKILL_FAILED` | `name`: the skill |
| `ROBOTIK_EVENT_ASSERTION_CHANGED` | `name`: the assertion, `flag`: 1 if it passes |

### `robotik::Canvas`

Valid only during the draw callback.

`text(line)`, `textColored(r, g, b, line)`, `bool button(label)`,
`bool checkbox(label, bool&)`, `bool radio(label, bool active)`,
`bool sliderInt(label, int&, min, max)`,
`separator()`. The `bool` results are true when the user acted.

---

## 2. `Mission`

Header: `Robotik/Runtime/Mission.hpp`. What your plugin adds to a generic
simulation. All methods are optional.

| Method | Called | Typical use |
|--------|--------|-------------|
| `setup(Simulation&, SceneView* view)` | Once, after the world is built, before the first reset. `view` is null in headless | Add skills, detectors, decorations; maybe your own backend |
| `reset(Simulation&, Seed)` | Each episode start, after the robot and objects are reset | Clear state; derive random streams from the seed |
| `step(Simulation&, Seconds dt)` | After each robot step | Command, measure, count |
| `measure(Simulation const&, Metrics&) const` | When assertions or reports need values | `metrics.set("name", value)` |
| `Status status(Simulation const&) const` | After each step, **only without behavior tree** | `SUCCESS` / `FAILURE` end the run; `RUNNING` / `IDLE` continue |

---

## 3. `Simulation`

Header: `Robotik/Runtime/Simulation.hpp`. Built by the host from a world and
a scenario; the mission receives it.

**The parts**

| Method | Returns |
|--------|---------|
| `robot()` | `RobotSession&`: joints, sensors, actuators, resources, kinematics |
| `skills()` | `SkillScheduler&` |
| `worldModel()` | `WorldModel&`: the beliefs |
| `perception()` | `PerceptionPipeline&`: add detectors here |
| `faults()` | `FaultInjector&` |
| `context()` | `RobotContext&`: what skills receive |
| `factory()` | `bt::NodeFactory&`: register custom tree nodes |
| `camera()` | `Camera*`: the first camera, or null |
| `scenario()` | `Scenario const&`: the parsed files (trajectories, world, …) |
| `mission()` | `Mission*` |
| `planning()` | `PlanningDomain&`: observers and bindings of the PDDL planner ([chapter 11](11-PDDL-Planner.md#5-reference)) |
| `planner()` | `PlanExecutor*`: the planner controller, or null |

**Time and run**

| Method | Does |
|--------|------|
| `time()` | Simulated time (`Seconds`) |
| `seed()` | Seed of the current episode |
| `reset(Seed)`, `reset()` | New episode (scenario seed by default) |
| `step(Seconds dt)` | One step (see [06-Architecture.md §3](06-Architecture.md#3-one-simulation-step)) |
| `finished()`, `outcome()` | Has the run ended, and why (`ControllerSucceeded`, `Timeout`, `StopCondition`, …) |
| `checks()`, `passed()` | The assertion verdicts |
| `oracle()` | True if the ground truth feeds the beliefs |

**Ground truth** (for metrics and assertions, never for skills)

| Method | Returns |
|--------|---------|
| `pose("id")`, `pose("id.link")` | Current true pose of an entity or a frame |
| `initialPose("id")` | Pose at the start of the episode |
| `query("distance(a, b)")` | Any value of the assertion grammar, or nothing if unknown |
| `evaluate("inside(a, b)")` | A full condition, with value and verdict |
| `entity("id")` | The Compages ECS entity, or an empty handle |
| `decorate(SceneObject)` | Adds a visual-only object (no collision, unknown to assertions) |

**Description** (what the world file says; for planning)

| Method | Returns |
|--------|---------|
| `described("id")` | Pose of the entity as written in the world file |
| `waypoints("name")` | A cartesian trajectory of the scenario, in the robot base frame |

---

## 4. Robot and joints

Headers: `Robotik/Robot/Robot.hpp`, `Robotik/Robot/Joints.hpp`.

### `Robot` (and `RobotSession`)

| Method | Returns / does |
|--------|----------------|
| `name()` | Name of the robot |
| `joints()` | `JointSet&` |
| `sensors()`, `actuators()` | Device sets, by name or by type |
| `resources()` | `ResourceManager&` |
| `time()` | Robot clock |
| `tool()` | Working frame: `tcp` with a tool, `tool0` without |
| `framePose("frame")` | **Forward kinematics**: pose of a frame in the robot base frame |
| `worldPose("frame")` | Same, in the world frame |
| `inverseKinematics("frame", Pose)` | **Inverse kinematics**: `std::optional<std::vector<double>>`, one value per joint |
| `kinematics()` | The `PinocchioBackend` (Jacobians…) |
| `base()` | State of a mobile base |

`RobotSession` adds `connect(backend)`, `hold(posture)`, `step(dt)`, `reset()`
(used by `Simulation`).

### `JointSet`

Two ways to name a joint:

```cpp
robotik::JointId const id = joints.find("joint1");      // index, doubles; NO_JOINT if absent
robotik::Revolute const j = joints.revolute("joint1");  // typed handle, Radians; false if absent
robotik::Prismatic const k = joints.prismatic("rail");  // typed handle, metres
```

| Method | Does |
|--------|------|
| `size()` | Number of joints; ids are `0 … size()-1` |
| `find(name)` / `require(name)` | Id, or `NO_JOINT` / throws |
| `revolute(name or id)`, `prismatic(name or id)` | Typed handle |
| `type(id)`, `isPrismatic(id)` | Kind of joint |
| `limits(handle)` | `lower`, `upper`, `bounded()` |
| `position(j)`, `velocity(j)`, `effort(j)` | Measured state |
| `home(j)`, `home(j, value)` | Read / set the home position |
| `moveTo(j, position)` | **Position command** (a target; the controller moves toward it) |
| `spin(j, velocity)` | Velocity command |
| `push(j, effort)` | Effort (torque / force) command |
| `reached(j, tolerance)` | In position mode and close to the target |
| `hold(j)`, `hold()` | Stay where you are |
| `release(j)` | Stop commanding (no effort) |
| `target(id)`, `mode(id)` | Current command |

---

## 5. Actuators

Header: `Robotik/Robot/Actuators.hpp`. Declared in the world file; each one is
also a resource.

```cpp
auto* group = p_simulation.robot().actuators().first<robotik::JointGroup>();
```

| Class | YAML `type` | Main methods |
|-------|-------------|--------------|
| `JointGroup` | `joint_group` | `joints()`, `contains(id)`, `moveTo(span<double>)`, `hold()`, `reached(tol)` |
| `Motor` | `motor` | `moveTo(Radians)`, `spin(AngularVelocity)`, `push(Torque)`, `stop()`, `position()` |
| `VacuumGripper` | `vacuum` | grasp / release what touches the suction face (see `plugins/PickAndPlace`) |

---

## 6. Skills

Headers: `Robotik/Skills/Skill.hpp`, `Robotik/Skills/MotionSkills.hpp`,
`Robotik/Runtime/RobotContext.hpp`.

```cpp
class Skill
{
    virtual void reset();
    virtual Status tick(RobotContext& p_context, Seconds p_dt) = 0;
    virtual void cancel(RobotContext& p_context);
    virtual std::string why() const;   // why the last run failed, for the user; empty by default
};

struct RobotContext { Robot& robot; WorldModel& world_model; Seconds time; Seconds dt; };
```

`Status`: `IDLE`, `RUNNING`, `SUCCESS`, `FAILURE`.

`SkillDescription`: `name`, `resources`, `priority`, `wait`, `cancellable`,
`preconditions` (`{ text, std::function<bool(RobotContext const&)> }`).

Built-in: `HomeSkill(group)`, `MoveJointSkill(joint, target)`,
`MoveJointsSkill(posture)`, `MoveTCPSkill(frame, target)` + `goal(pose)`,
`StopSkill`. See [05-Skills-and-Behavior-Trees.md](05-Skills-and-Behavior-Trees.md).

---

## 7. `SkillScheduler`

Header: `Robotik/Runtime/Scheduler.hpp`.

| Method | Does |
|--------|------|
| `add<T>(description, args...)` | Registers a skill; returns its `SkillId`. Throws on a duplicate name |
| `find(name)` | `SkillId`, or `NO_SKILL` |
| `request(id)` | Ask for a run (no effect if already waiting or running) |
| `cancel(id)`, `cancelAll()` | Cancel at the next update |
| `update(context)` | Admit, preempt, tick (the simulation calls it) |
| `state(id)`, `reason(id)`, `blocker(id)` | `SkillState`, `SkillReason`, the resource behind it |
| `precondition(id)` | The false precondition, if blocked by one |
| `name(id)`, `description(id)`, `skill(id)` | Introspection |
| `trace()` | Every run since the last reset (`SkillRun`: state, reason, start, end) |

---

## 8. Resources and faults

Headers: `Robotik/Runtime/Resources.hpp`, `Robotik/Runtime/Faults.hpp`.

| `ResourceManager` | Does |
|-------------------|------|
| `require(name, Access)` | A `ResourceRequirement` for a `SkillDescription`. `Access::Exclusive` (default) or `Access::Shared` |
| `find(name)`, `name(id)`, `size()` | Lookup |
| `available(name or id)` | False while in fault |
| `fail(name)`, `restore(name)`, `restoreAll()` | Inject / clear a fault by hand |

`FaultInjector` applies the `faults:` of the scenario (`at`, `rate`) at each
step. You rarely need it directly.

---

## 9. Perception and beliefs

Headers: `Robotik/Perception/Detector.hpp`, `Detection.hpp`, `WorldModel.hpp`,
`Robotik/Sensors/Camera.hpp`.

**Write a detector:**

```cpp
class RedBlobDetector final: public robotik::Detector
{
public:
    void detect(robotik::CameraFrame const& p_frame, robotik::Detections& p_out) override
    {
        // read p_frame.rgb, p_frame.depth, p_frame.intrinsics, p_frame.pose; push what you find:
        robotik::Detection d;
        d.label = "red_cube";
        d.confidence = 0.9f;
        d.center = { 120.0f, 80.0f };          // pixels
        // d.position = ... (optical frame, metres) if you know the depth
        p_out.items.push_back(d);
    }
};

// in Mission::setup:
p_simulation.perception().add<RedBlobDetector>();
```

`Detection`: `label`, `id` (fiducial id or -1), `confidence`, `box` (pixels),
`center` (pixels), `position` (optional, optical frame), `pose` (optional).

**Read beliefs** (in a skill, through `p_context.world_model`):

| `WorldModel` | Does |
|--------------|------|
| `find(name)` | `WorldObject*` or null |
| `objects()` | All beliefs |
| `observe(name, point, confidence, stamp)` | Update a belief |
| `add(name, position, size)`, `place(...)` | Declare / move a belief |

`WorldObject`: `name`, `position`, `size`, `confidence`, `seen` (time),
`observations`, `observed()`, `top()`.

Other sensors (`Robotik/Sensors/`): `Imu`, `RangeScanner`,
`ForceTorqueSensor`. All derive from `Sensor` (`name()`, `frequency()`,
`samples()`, `stamp()`), and their readings derive from `Measurement`
(time stamp, frame, sequence, validity).

---

## 10. Metrics

Header: `Robotik/Runtime/Metrics.hpp`.

```cpp
void measure(robotik::Simulation const&, robotik::Metrics& p_metrics) const override
{
    p_metrics.set("laps", m_laps);
    p_metrics.set("cross_track.max", m_max_error);
}
```

Any published name can be used in an assertion (`laps >= 1`) or in
`evaluation.metrics` of an experiment. Use a prefix per plugin
(`hello.ticks`, `reach.error`) to avoid clashes.

---

## 11. Assets

Header: `Robotik/Assets/AssetResolver.hpp`.

```cpp
robotik::AssetResolver const assets = robotik::AssetResolver::standard();
std::filesystem::path const urdf = assets.model("robotik.robots.robot_6axis");
```

| Method | Does |
|--------|------|
| `standard()` | Resolver with the standard search roots (`$ROBOTIK_DATA`, `data/packs`, install prefix) |
| `model(id)` | Path of a model; throws if unknown |
| `find(pack id)`, `require(pack id, version)` | A pack |
| `addRoot(path)`, `addPack(path)` | More places to search |
| `packs()`, `errors()` | What was found, and problems (conflicts) |

---

## 12. Learning

Headers in `Robotik/Environment/`. See [08-Reinforcement-Learning.md](08-Reinforcement-Learning.md).

| Class | Role |
|-------|------|
| `Environment` | `reset(seed, observation)`, `step(action, observation)`, `observationSpace()`, `actionSpace()` |
| `Space` | Named, bounded dimensions (`add`, `add3`), with units; `privileged()` |
| `SimulationEnvironment` | A scenario + `ObservationBuilder` + `ActionAdapter` + `RewardEvaluator` |
| `EnvironmentPool` | N environments on threads, derived seeds |
| `EpisodeLog` | JSON log of the episodes |
| `Agent` (`Robotik/Agents/Agent.hpp`) | `Action update(Observation const&, Seconds)`: any brain |

---

## 13. Scenario objects

Headers in `Robotik/Scenario/`. Usually built by the host; useful in tests.

| Class | Role |
|-------|------|
| `Scenario::load(path, assets)` | Parse a scenario and its world |
| `ScenarioSource`, `ConfigNode` | The YAML as an editable tree; `override("world.entities.x.y", ...)` then `Scenario::read` |
| `Experiment` | Cases, seeds, evaluation |
| `WorldDescription`, `EntitySpec` | The parsed world |
| `Trajectory` | A cartesian or joint path |
| `AssertionEngine`, `Check` | Verdicts: `verdict`, `text`, `detail`, `ok()` |
