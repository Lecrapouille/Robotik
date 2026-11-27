# 📘 API reference

[⏫ Reference](README.md)

The classes a plugin author uses, grouped by task. For each one: the header, what it is for, and its main methods. The headers have more details in their comments; this page tells you where to look.

Conventions in the code:

- namespace `robotik`; units from `RobotIK/Common/Math.hpp` (`Seconds`, `Radians`, …), the same types as Compages (`compages::si`);
- headers are in three layers (see [Architecture](../concepts/architecture/README.md) and the [glossary](glossary.md)): `RobotIK/Robot/` (one robot), `RobotIK/Task/` (a job in C++), `RobotIK/Package/` (files and plugins), plus `RobotIK/Common/`;
- reading a file returns a `Result` (`if (!r) … r.error()`, then `r.value()` or `r.take()`); a mistake in the code throws;
- parameters start with `p_`, members with `m_`;
- SI units everywhere: metres, radians, seconds, newtons;
- z up, like URDF.

| I want to… | Use | Section |
|------------|-----|---------|
| Write the entry point of my plugin | `Plugin`, `Host`, `Canvas` | [Plugin API](#-plugin-api) |
| Add behaviour to a simulation | `ScenarioLogic` | [`ScenarioLogic`](#-scenariologic) |
| Read the world, the time, the truth | `RoboticCell` (and `Simulation` for the verdict) | [`RoboticCell` and `Simulation`](#-roboticcell-and-simulation) |
| Move joints, read poses, compute IK | `Robot`, `JointSet` | [Robot and joints](#-robot-and-joints) |
| Use the gripper, a motor | actuators | [Actuators](#-actuators) |
| Write a skill | `Skill`, `SkillDescription`, `RobotContext` | [Skills](#-skills) |
| Run, cancel, inspect skills | `SkillScheduler` | [`SkillScheduler`](#-skillscheduler) |
| Reserve parts of the robot, inject faults | `ResourceManager`, `FaultInjector` | [Resources and faults](#-resources-and-faults) |
| Process images, update beliefs | `Detector`, `PerceptionPipeline`, `WorldModel` | [Perception and beliefs](#-perception-and-beliefs) |
| Publish numbers for assertions | `Metrics` | [Metrics](#-metrics) |
| Find a 3D model | `AssetResolver` | [Assets](#-assets) |
| Make an RL environment | `Environment`, `SimulationEnvironment` | [Learning](#-learning) |

One umbrella header includes everything: `#include "RobotIK/RobotIK.hpp"`.

## 🔌 Plugin API

Header: `RobotIK/Package/Plugin/PluginAPI.hpp` (C++ helper over the C border `RobotIK/Package/Plugin/PluginABI.h`).

### 🔌 `robotik::ScenarioPlugin`

The common base: one scenario logic per scenario.

```cpp
class PainterPlugin final: public robotik::ScenarioPlugin
{
public:
    static robotik::PluginInfo describe() { return { .id = "robotik.painter" }; }
    PainterPlugin()
        : ScenarioPlugin({ { "painter", robotik::logic<PainterLogic>(false) },
                          { "painter_pddl", robotik::logic<PainterLogic>(true) } }) {}
};
ROBOTIK_EXPORT_PLUGIN(PainterPlugin)
```

| Member | Does |
|--------|------|
| `ScenarioPlugin({ { id, factory }, ... })` | The scenario logic of each scenario id of `plugin.yaml` |
| `robotik::logic<T>(args...)` | A factory building `T(args...)` |
| `setup(host, scenario)` | Creates and binds the scenario logic; an unknown id throws (with the known ids). Override it and call `ScenarioPlugin::setup` first to add menus and panels |
| `shutdown(host)` | Frees the scenario logic |
| `scenario()` | Id of the current scenario |
| `logic<T>()` | The current scenario logic as a `T`, or null |

### 🔌 `robotik::Plugin`

Base class of `ScenarioPlugin`, for a plugin that chooses its scenario logic another way. Override only what you need; every method does nothing by default.

| Method | Called |
|--------|--------|
| `static PluginInfo describe()` | Before the instance exists. Return `{ .id = "..." }`, the same id as `plugin.yaml` |
| `setup(Host&, std::string_view scenario)` | Before the simulation exists. Create the scenario logic, `bindLogic`, menus, panels |
| `start(Host&)` | The run starts |
| `update(Host&, Seconds dt)` | After each step (or instead of it with `host.clock: plugin`) |
| `onEvent(Host&, RobotIKEvent const&)` | A subscribed event |
| `pause(Host&, bool paused)` | Pause / Play |
| `stop(Host&)` | End of the run; the simulation still exists |
| `shutdown(Host&)` | Free the scenario logic; the simulation is destroyed |

Then, once, in one `.cpp` file: `ROBOTIK_EXPORT_PLUGIN(MyPlugin)`.

A method that cannot go on throws. The exception is caught at the C border and reported to the user; the host gets `ROBOTIK_PLUGIN_ERR_SETUP` from `setup`, `ROBOTIK_PLUGIN_ERR_RUNTIME` from the others.

### 🏠 `robotik::Host`

| Method | Does |
|--------|------|
| `bindLogic(ScenarioLogic&)` | The scenario logic the simulation will use. Call it in `setup` |
| `Simulation* simulation()` | The running simulation, or null |
| `log(level, text)`, `error(text)` | Messages. Levels `ROBOTIK_LOG_INFO / _WARNING / _ERROR` |
| `subscribe(RobotIKEventKind)` | Receive this kind of event |
| `addMenuItem(menu, item, std::function<void()>)` | Simulator menu entry; returns a handle |
| `addPanel(id, title, std::function<void(Canvas const&)>)` | Simulator panel; returns a handle |
| `removeUi(handle)` | Remove a menu entry or a panel |
| `scenarioPath()` | Path of the scenario file |

`Host` is a small value (a pointer); you may keep a copy in your plugin.

### 📨 `RobotIKEvent`

| `kind` | Fields |
|--------|--------|
| `ROBOTIK_EVENT_KEY_PRESSED` | `code`: the key (`'H'`) |
| `ROBOTIK_EVENT_SCENARIO_STARTED`, `_STOPPED` | — |
| `ROBOTIK_EVENT_SKILL_COMPLETED`, `_SKILL_FAILED` | `name`: the skill |
| `ROBOTIK_EVENT_ASSERTION_CHANGED` | `name`: the assertion, `flag`: 1 if it passes |

### 🖌 `robotik::Canvas`

Valid only during the draw callback.

`text(line)`, `textColored(r, g, b, line)`, `bool button(label)`, `bool checkbox(label, bool&)`, `bool radio(label, bool active)`, `bool sliderInt(label, int&, min, max)`, `separator()`. The `bool` results are true when the user acted.

## 🎯 `ScenarioLogic`

Header: `RobotIK/Task/ScenarioLogic.hpp`. What your plugin adds to a generic simulation. All methods are optional.

| Method | Called | Typical use |
|--------|--------|-------------|
| `setup(RoboticCell&, SceneView* view)` | Once, after the world is built, before the first reset. `view` is null in headless | Add skills, detectors, decorations; maybe your own backend |
| `reset(RoboticCell&, Seed)` | Each episode start, after the robot and objects are reset | Clear state; derive random streams from the seed |
| `act(RobotContext&, Seconds dt)` | After each robot step | Command the robot from what it measures and believes; no ground truth |
| `simulate(RoboticCell&, Seconds dt)` | After `act`, in simulation only | What the world does by itself: a conveyor, ink on a canvas |
| `observe(RoboticCell const&, Seconds dt)` | After `simulate` | Read the truth and count for the metrics; a const cell commands nothing |
| `measure(RoboticCell const&, Metrics&) const` | When assertions or reports need values | `metrics.set("name", value)` |
| `Status status(RobotContext const&) const` | After each step, **only without behavior tree** | `SUCCESS` / `FAILURE` end the run; `RUNNING` / `IDLE` continue |

## 🏭 `RoboticCell` and `Simulation`

Header: `RobotIK/Task/RoboticCell.hpp`. The robot cell: robots, objects, one physics, the controller and the beliefs. The scenario logic receives it. A C++ program can build one from a `RoboticCellDescription`, without any file ([Three programs, one per layer](../tutorials/02-three-programs.md)). `RobotIK/Task/OpenRoboticCell.hpp` builds the cell and its world in one call: `robotik::openRoboticCell(description, &logic)` returns a `Result<OpenedRoboticCell>`. `RobotIK/Package/Simulation.hpp` builds the cell from a scenario file and judges the run (marked *Simulation* below).

**The parts**

| Method | Returns |
|--------|---------|
| `robot()` | `Robot&` of the first robot, as its skills see it: joints, sensors (read only), actuators, kinematics |
| `robot(id)`, `robots()` | One robot by its entity id (or null), every robot in file order (`Robot const*`) |
| `runtime()`, `runtime(id)`, `runtimes()` | The same robots, as `RobotRuntime`: true base, backend, entities, writable sensors (for `simulate` and `observe`) |
| `skills()` | `SkillScheduler&` |
| `worldModel()` | `WorldModel&`: the beliefs |
| `perception()` | `PerceptionPipeline&`: add detectors here |
| `faults()` | `FaultInjector&` |
| `context()`, `context(id)` | `RobotContext&`: what skills receive (first robot, or robot `id`) |
| `beliefFrame()` | Frame of the beliefs: the robot base when alone, the world with several robots |
| `physicsOptions()` | Options of the physics the core builds after `ScenarioLogic::setup`: tune them there (floating base, gravity compensation, step, `armature`, `damping`, `exact_inertia`); the world still adds its bodies and floor (MicroDuck) |
| `worldFrame()` | Entity of the Z-up frame of the world: hang on it what stays in the world, not on a robot root that a floating base moves |
| `factory()` | `bt::NodeFactory&`: register custom tree nodes (include `RobotIK/Task/Skills/SkillNodes.hpp`) |
| `camera()` | `Camera*`: the first camera, or null |
| `description()` | `RoboticCellDescription const&`: what the cell was built from (world, trajectories, parameters, …) |
| `logic()` | `ScenarioLogic*` |
| `planning()` | `PlanningDomain&`: observers and bindings of the PDDL planner ([The PDDL planner: reference](pddl-planner.md)) |
| `planner()` | `PlanExecutor*`: the planner controller, or null |

**Time and run**

| Method | Does |
|--------|------|
| `run(skill, limit)` | Requests the skill and steps until it ends; returns its `SkillState` |
| `run(limit)` | Steps until the controller ends (tree, plan or scenario logic status); returns `Status` |
| `achieve(PlannerSpec)` | Plans toward the goal and runs the plan (a `planner:` block does the same); throws when domain, goal and bindings disagree |
| `status()` | `robotik::Status` of the controller (`IDLE` before the first tick) |
| `time()` | Simulated time (`Seconds`) |
| `seed()` | Seed of the current episode |
| `reset(Seed)`, `reset()` | New episode (scenario seed by default) |
| `step(Seconds dt)` | One step (see [One simulation step](../concepts/architecture/01-decision-path.md#-one-simulation-step)) |
| `status()`, `stopController()` | State of the controller (tree, plan or scenario logic); stop ticking it |
| `perceptionOf("id")`, `perceptionSummary()` | Perception fidelity of an object (`truth`, `noisy_truth`, `sensed`) and of the run |
| *Simulation* `finished()`, `outcome()` | Has the run ended, and why (`ControllerSucceeded`, `Timeout`, `StopCondition`, …) |
| *Simulation* `checks()`, `passed()` | The assertion verdicts |
| *Simulation* `ended`, `assertionChanged` | Signals: the run ended, an assertion changed color |

**Ground truth** (for metrics and assertions, never for skills)

| Method | Returns |
|--------|---------|
| `pose("id")`, `pose("id.link")` | Current true pose of an entity or a frame |
| `initialPose("id")` | Pose at the start of the episode |
| `query("(distance a b)")` | Any value of a condition; throws on an unknown name or entity |
| `query("inside", { "a", "b" })` | A world query from C++, without text |
| `addQuery({ name, min_args, max_args, answer })` | A query of the plugin, usable in conditions |
| `compile("(inside a b)")` | A condition read once (`Result<Condition>`), then `robotik::evaluate(c, cell.metrics())` |
| `evaluate("(inside a b)")` | A full condition now, with value and verdict |
| `entity("id")` | The Compages ECS entity, or an empty handle |
| `decorate(Shape)` | Adds a visual-only object (no collision, unknown to assertions) |

**Description** (what the world file says; for planning)

| Method | Returns |
|--------|---------|
| `described("id")` | Pose of the entity as written in the world file |
| `waypoints("name")` | A cartesian trajectory of the scenario, in the robot base frame |

## 🤖 Robot and joints

Headers: `RobotIK/Robot/Robot.hpp` (skills), `RobotIK/Robot/RobotRuntime.hpp` (the program that runs the robot), `RobotIK/Robot/RobotModel.hpp`, `RobotIK/Robot/Joints.hpp`.

### 🤖 `Robot` (what a skill gets)

| Method | Returns / does |
|--------|----------------|
| `getName()` | Name of the robot (URDF); `getId()` its id in the world |
| `getJoints()` | `JointSet&` |
| `getSensors()` | `SensorSet const&`: find and read, never change a sensor |
| `getActuators()` | `ActuatorSet&`: find and command (no `add`) |
| `getModel()` | `RobotModel const&`: name, URDF, work frame, joint names (does not change) |
| `getResources()` | `ResourceManager const&` |
| `getTime()` | Robot clock |
| `getTool()` | Working frame: `tcp` with a tool, `tool0` without |
| `requireFramePose("frame")` | **Forward kinematics**: pose of a frame in the robot base frame |
| `requireWorldPose("frame")` | Same, in the world frame |
| `inverseKinematics("frame", Pose[, IkGoal])` | **Inverse kinematics**: `std::optional<std::vector<double>>`, one value per joint. `IkGoal::Position` or `Orientation` places only that part, for a link with fewer than six joints above it (the closest reachable one) |
| `getKinematics()` | The `Kinematics`, read only (Jacobians…) |
| `getBase()` | Estimated state of a mobile base |
| `requireLink("frame")`, `getWorldPose(link)` | A link found once, then its pose with no search |

### ⚙ `RobotRuntime` (what runs the robot)

`RobotRuntime` owns the `Robot` and lends it with `robot()`: it is not a base class of it, so a skill cannot cast its `Robot&` to the runtime. The `Robot` does not store the world, the entities, the backend nor the true base: they are in the runtime only.

| Method | Returns / does |
|--------|----------------|
| `robot()` | The `Robot&` to give to the skills |
| `connect(backend)`, `step(dt)` (= `prepare`, `advance`, `sense`), `reset()`, `hold(posture)`, `setStartPose(pose)` | Run the robot (used by `RoboticCell`) |
| `addSensor<T>(name, …)`, `addActuator<T>(name, …)` | The only way to add a device |
| `getSensors()` | Writable `SensorSet&`: image source, noise seed, readings |
| `getResources()`, `getKinematics()` | Writable (faults, tool change) |
| `measureJoint(joint, q, v)`, `placeJoint(joint, q)` | What the encoders read (typed: a `Revolute` takes `Radians`, a `Prismatic` takes `Length`) |
| `measureHolding(cup, on)`, `measureBase(state)`, `estimateBase(state)` | What the vacuum sensor, the backend and a localization write |
| `getTrueBase()`, `getTrueWorldPose(link)`, `requireTrueWorldPose("frame")` | The truth, for sensors and checks |
| `getWorld()`, `getRoot()`, `getLink("name")`, `getJointLink(id)`, `getUrdfFrame()` | ECS entities (rendering) |
| `getBackend()` | The `RobotBackend`, or null |
| `getPhysics()` | The `PhysicsWorld` (rays, contacts, body poses, welds), null on hardware |
| `getHealth()` | `RobotBackend::Health` after the last step: while not `ok`, the actuators are failed resources |
| `setId(id)`, `setLocalization(fidelity)` | Identity and who writes the estimated base |

`make check-layers` compiles `Task/Skills` and `Task/Planning` without `RobotRuntime.hpp`; `tests/Robot/TestSkillAccess.cpp` checks with the compiler that a `Robot&` cannot change a sensor, add a device, reach an entity, nor command a robot of `RobotContext::team`. Sensors (`update(RobotRuntime&, now)`), backends and grasp models receive the runtime.

In a `RoboticCell`, `robot()`, `robot(id)` and `robots()` give the `Robot` of the skills; `runtime()`, `runtime(id)` and `runtimes()` give what runs them.

### 🔌 `RobotBackend` and `PhysicsWorld`

`RobotBackend` is the hardware abstraction layer of one robot: `attach`, `reset`, `step(dt)`, `time()`, `getHealth()`. A driver for real hardware implements it alone. `PhysicsWorld` is what only a simulator knows: `raycast`, `wrench`, `bodyPose`, `contactList`, `contacts`, `weld`, `placeBody`. `MujocoBackend` implements both.

### 🧮 `JointSet`

Two ways to name a joint:

```cpp
robotik::JointId const id = joints.find("joint1");      // index, doubles; NO_JOINT if absent
robotik::Revolute const j = joints.requireRevolute("joint1");  // typed handle, Radians; throws if absent
robotik::Prismatic const k = joints.requirePrismatic("rail");  // typed handle, metres
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

## 🦾 Actuators

Header: `RobotIK/Robot/Actuators.hpp`. Declared in the world file; each one is also a resource.

```cpp
auto* group = p_simulation.robot().getActuators().first<robotik::JointGroup>();
```

| Class | YAML `type` | Main methods |
|-------|-------------|--------------|
| `JointGroup` | `joint_group` | `joints()`, `contains(id)`, `moveTo(span<double>)`, `hold()`, `reached(tol)` |
| `Motor` | `motor` | `moveTo(Radians)`, `spin(AngularVelocity)`, `push(Torque)`, `stop()`, `position()` |
| `VacuumGripper` | `vacuum` | grasp / release what touches the suction face (see `demos/PickAndPlace`) |

## 🔧 Skills

Headers: `RobotIK/Task/Skills/Skill.hpp`, `RobotIK/Task/Skills/MotionSkills.hpp`, `RobotIK/Task/RobotContext.hpp`.

```cpp
class Skill
{
    virtual void reset();
    virtual Status tick(RobotContext& p_context, Seconds p_dt) = 0;
    virtual void cancel(RobotContext& p_context);
    SkillFailure const& failure() const;   // why the current run failed: { code, why }; empty until it fails
protected:
    Status fail(std::string code, std::string why);  // records the failure, returns FAILURE
    Status fail(SkillFailure child);                 // passes on the failure of a child skill
};

struct SkillFailure { std::string code; std::string why; };  // code for the tree, why for a person

struct RobotContext { Robot& robot; WorldModel& world_model; Seconds time; Seconds dt;
                      std::span<Robot* const> team; };   // team: every robot, empty when alone
```

`Status`: `IDLE`, `RUNNING`, `SUCCESS`, `FAILURE`.

`SkillDescription`: `name`, `resources`, `priority`, `wait`, `cancellable`, `preconditions` (`{ text, std::function<bool(RobotContext const&)> }`), `robot` (id of the robot it drives; empty: the first one).

Built-in: `HomeSkill(group)`, `MoveJointSkill(joint, target)`, `MoveJointsSkill(posture)`, `MoveTCPSkill(frame, target)` + `goal(pose)`, `StopSkill`. See [Skills, the scheduler and behavior trees](../concepts/04-skills-and-behavior-trees.md).

## 📅 `SkillScheduler`

Header: `RobotIK/Task/Scheduler.hpp`.

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

## 🔒 Resources and faults

Headers: `RobotIK/Robot/Resources.hpp`, `RobotIK/Task/Faults.hpp`.

| `ResourceManager` | Does |
|-------------------|------|
| `require(name, Access)` | A `ResourceRequirement` for a `SkillDescription`. `Access::Exclusive` (default) or `Access::Shared` |
| `find(name)`, `name(id)`, `size()` | Lookup |
| `available(name or id)` | False while in fault |
| `fail(name)`, `restore(name)`, `restoreAll()` | Inject / clear a fault by hand |

`FaultInjector` applies the `faults:` of the scenario (`at`, `rate`) at each step. You rarely need it directly.

## 👁 Perception and beliefs

Headers: `RobotIK/Task/Perception/Detector.hpp`, `Detection.hpp`, `WorldModel.hpp`, `RobotIK/Robot/Sensors/Camera.hpp`.

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

// in ScenarioLogic::setup:
p_simulation.perception().add<RedBlobDetector>();
```

`Detection`: `label`, `id` (fiducial id or -1), `confidence`, `box` (pixels), `center` (pixels), `position` (optional, optical frame), `pose` (optional).

**Read beliefs** (in a skill, through `p_context.world_model`):

| `WorldModel` | Does |
|--------------|------|
| `find(name)` | `WorldObject*` or null |
| `objects()` | All beliefs |
| `observe(name, point, confidence, stamp)` | Update a belief |
| `add(name, position, size)`, `place(...)` | Declare / move a belief |

`WorldObject`: `name`, `position`, `size`, `confidence`, `seen` (time), `observations`, `observed()`, `top()`.

Other sensors (`RobotIK/Robot/Sensors/`): `Imu`, `RangeScanner`, `ForceTorqueSensor`. All derive from `Sensor` (`name()`, `frequency()`, `samples()`, `stamp()`), and their readings derive from `Measurement` (time stamp, frame, sequence, validity).

## 📊 Metrics

Header: `RobotIK/Task/Metrics.hpp`.

```cpp
void measure(robotik::RoboticCell const&, robotik::Metrics& p_metrics) const override
{
    p_metrics.set("laps", m_laps);
    p_metrics.set("cross_track.max", m_max_error);
}
```

Any published name can be used in an assertion (`(>= laps 1)`) or in `evaluation.metrics` of an experiment. Use a prefix per plugin (`hello.ticks`, `reach.error`) to avoid clashes.

## 📦 Assets

Header: `RobotIK/Robot/Assets/AssetResolver.hpp`.

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

## 🎲 Learning

Headers in `RobotIK/Task/Learning/`. See [Build reinforcement learning environments](../guides/reinforcement-learning.md).

| Class | Role |
|-------|------|
| `Environment` | `reset(seed, observation)`, `step(action, observation)`, `observationSpace()`, `actionSpace()` |
| `Space` | Named, bounded dimensions (`add`, `add3`), with units; `privileged()` |
| `SimulationEnvironment` | A cell description (a scenario is one) + `ObservationBuilder` + `ActionAdapter` + `RewardEvaluator` |
| `EnvironmentPool` | N environments on threads, derived seeds |
| `EpisodeLog` | JSON log of the episodes |
| `Agent` (`RobotIK/Task/Agents/Agent.hpp`) | `Action update(Observation const&, Seconds)`: any brain |

## 🎬 Scenario objects

Headers in `RobotIK/Package/Scenario/`. Usually built by the host; useful in tests.

| Class | Role |
|-------|------|
| `loadScenario(path, assets)` | Parse a scenario and its world; returns a `Result<Scenario>` |
| `ScenarioSource`, `ConfigNode` | The YAML as an editable tree: `loadScenarioSource`, `overrideValue(source, "world.entities.x.y", ...)`, then `readScenario` |
| `Experiment` | Cases, seeds, evaluation: `loadExperiment`, `scenarioOf(experiment, case, assets)` |
| `WorldDescription`, `EntitySpec` | The parsed world (Task layer data: `findEntity`, `firstRobot`, `robotsOf`) |
| `Trajectory` | A cartesian or joint path: `loadTrajectory` |
| `AssertionEngine`, `Check` | Verdicts: `verdict`, `text`, `detail`, and `ok(check)` |
