# 4. Your first plugin

A **plugin** is your program. It is a folder in `plugins/` with some YAML
files (world, scenarios) and a little C++ compiled into a shared library
(`.so`). The simulator and Headless load it when you choose one of its
scenarios.

This page follows the **HelloWorld** plugin, lesson by lesson, then shows how
to create your own plugin from zero.

| Lesson | Scenario | You learn |
|--------|----------|-----------|
| 1 | `hello.scenario.yaml` | The smallest mission: count steps, publish a metric, end the run |
| 2 | `wave.scenario.yaml` | Command a joint, read the tool pose (forward kinematics) |
| 3 | `reach.scenario.yaml` | Reach a point with inverse kinematics |
| 4 | `skills.scenario.yaml` | Write a skill, call it from a behavior tree |
| — | all | The plugin class: menus, panels, keyboard |

Run them first, to know what you are reading:

```bash
./build/linux-gcc-debug/bin/Robotik-Simulator          # Demos → Hello world → ...
./build/linux-gcc-debug/bin/Robotik-Headless build/linux-gcc-debug/lib/robotik/plugins/HelloWorld/scenarios/hello.scenario.yaml
```

---

## 1. What is in a plugin?

```text
plugins/HelloWorld/
├── plugin.yaml          the manifest: name, library, scenarios
├── Makefile             builds librobotik_hello_world.so
├── src/                 the C++ code
│   ├── Plugin.cpp           the plugin class: chooses a mission per scenario
│   ├── HelloMission.*       lesson 1
│   ├── WaveMission.*        lesson 2
│   ├── ReachMission.*       lesson 3
│   └── WaveSkill.*          lesson 4
├── worlds/arm.world.yaml      what exists: one 6-axis arm
├── scenarios/           what happens and what is checked
└── experiments/         variants to compare
```

**YAML** says *what*: the world, the task, the checks. **C++** says *how*:
the behaviours the YAML cannot express.

Who calls what:

```text
 Robotik-Simulator / Robotik-Headless   (the "host")
        │ 1. reads plugin.yaml (no code loaded yet)
        │ 2. loads librobotik_hello_world.so when you pick a scenario
        ▼
 HelloWorldPlugin::setup("wave")   ← your Plugin class
        │ creates a WaveMission, gives it to the host
        ▼
 Simulation (built from worlds/arm.world.yaml + scenarios/wave.scenario.yaml)
        │ at each step:
        ▼
 WaveMission::step()               ← your Mission class
```

So there are **two classes** to know:

| Class | Role | Talks to |
|-------|------|----------|
| `robotik::Plugin` | The entry point. Chooses the mission, adds menus and panels | The host (window, keyboard) |
| `robotik::Mission` | The behaviour during the simulation: skills, metrics, end | The simulation (robot, world) |

---

## 2. Lesson 1 — the smallest mission

`scenarios/hello.scenario.yaml`:

```yaml
scenario:
  id: hello
  description: Count the simulation steps, then stop after two seconds.
  world: worlds/arm.world.yaml      # relative to the package root
  seed: 1
  timeout: 30
  assertions:
    - controller.success      # the mission said SUCCESS
    - hello.ticks > 100       # a metric published by the mission
    - time < 3
```

`src/HelloMission.hpp`:

```cpp
class HelloMission final: public robotik::Mission
{
public:

    void reset(robotik::Cell& p_cell, robotik::Seed p_seed) override;
    void step(robotik::Cell& p_cell, Seconds p_dt) override;
    void measure(robotik::Cell const& p_cell,
                 robotik::Metrics& p_metrics) const override;
    [[nodiscard]] robotik::Status status(robotik::Cell const& p_cell) const override;

private:

    double m_ticks = 0.0;
};
```

`src/HelloMission.cpp`:

```cpp
void HelloMission::reset(robotik::Cell& /*p_cell*/, robotik::Seed /*p_seed*/)
{
    m_ticks = 0.0;                         // a new episode starts
}

void HelloMission::step(robotik::Cell& /*p_cell*/, Seconds /*p_dt*/)
{
    m_ticks += 1.0;                        // called after each simulation step
}

void HelloMission::measure(robotik::Cell const& /*p_cell*/,
                           robotik::Metrics& p_metrics) const
{
    p_metrics.set("hello.ticks", m_ticks); // the scenario can now use "hello.ticks"
}

robotik::Status HelloMission::status(robotik::Cell const& p_cell) const
{
    if (p_cell.time() >= Seconds(2.0))
    {
        return robotik::Status::SUCCESS;   // ends the run: controller.success = 1
    }
    return robotik::Status::RUNNING;
}
```

The five methods of a `Mission`, all optional:

| Method | When | Use it to |
|--------|------|-----------|
| `setup(simulation, view)` | Once, after the world is built | Add skills, detectors, decorations. `view` is null in headless |
| `reset(simulation, seed)` | At each episode start | Clear your state. Derive random numbers from `seed` |
| `step(simulation, dt)` | After each simulation step | Command the robot, update your state |
| `measure(simulation, metrics)` | When the assertions need values | Publish named numbers |
| `status(simulation)` | After each step, **only if the scenario has no controller** (behavior tree, planner or GRAFCET) | Return `SUCCESS` or `FAILURE` to end the run |

> **Rule:** the plugin does not judge itself. It publishes metrics; the
> scenario file decides what is a success. Do not write `assert` or
> `is_finished()` in C++: write an assertion in YAML.

---

## 3. Lesson 2 — move a joint (forward kinematics)

The arm turns its base joint left and right. `src/WaveMission.cpp`:

```cpp
void WaveMission::step(robotik::Cell& p_cell, Seconds /*p_dt*/)
{
    robotik::Robot& robot = p_cell.robot();
    robotik::JointSet& joints = robot.joints();

    // A typed handle: the angle is in radians, the compiler checks it.
    robotik::Revolute const base = joints.revolute("joint1");
    if (!base)
    {
        return;
    }

    // Command: a target angle. The joint controller moves toward it.
    Radians const home = joints.home(base);
    Radians const offset(0.6 * std::sin(p_cell.time().value()));
    joints.moveTo(base, home + offset);

    // Measure: the angle reached, and where the tool is (forward kinematics,
    // in the robot base frame).
    m_swing = std::max(m_swing, std::abs((joints.position(base) - home).value()));
    m_tool_y = robot.framePose(robot.tool()).position.y;
}
```

What you see here:

- `p_cell.robot()` gives the robot. `robot.joints()` gives all joints.
- `joints.revolute("joint1")` gives a **typed handle**: its positions are
  `Radians`. A prismatic joint gives a `Prismatic` handle in metres. Mixing
  them does not compile.
- `moveTo` gives a **target**; the joint does not jump there. The physics
  (MuJoCo) moves it at a realistic speed. This is why we measure the
  *reached* angle with `joints.position(base)`, not the commanded one.
- `robot.tool()` is the name of the working frame: `tcp` when a tool is
  mounted, `tool0` otherwise. `robot.framePose(frame)` is **forward
  kinematics**: the pose of that frame in the robot base frame.

The scenario checks the result and the safety:

```yaml
  assertions:
    - controller.success
    - wave.swing > 0.4
    - { condition: "joint_limits_respected(robot)", phase: always }
```

**Try it:** change `0.6` into `0.2` in the code, rebuild
(`make robotik_hello_world`), run again. `wave.swing > 0.4` now fails, and
the report tells you the joint swung only 0.2 rad.

---

## 4. Lesson 3 — reach a point (inverse kinematics)

Now we give a **position** and ask which joint angles reach it.
`src/ReachMission.cpp`:

```cpp
void ReachMission::reset(robotik::Cell& p_cell, robotik::Seed /*p_seed*/)
{
    robotik::Robot& robot = p_cell.robot();

    // The tool frame is "tcp" when a tool is mounted, "tool0" otherwise.
    robotik::Pose const start = robot.framePose(robot.tool());
    m_target = start;
    m_target.position.x += 0.03;                      // 3 cm forward

    // Inverse kinematics: no answer when the pose is out of reach.
    std::optional<std::vector<double>> const solution =
        robot.inverseKinematics(robot.tool(), m_target);
    m_solution = solution ? *solution : std::vector<double>{};
    m_error = distance(start, m_target);
}

void ReachMission::step(robotik::Cell& p_cell, Seconds /*p_dt*/)
{
    robotik::Robot& robot = p_cell.robot();
    robotik::JointSet& joints = robot.joints();
    for (robotik::JointId id = 0; id < joints.size() && id < m_solution.size(); ++id)
    {
        joints.moveTo(id, m_solution[id]);            // one target per joint
    }
    m_error = distance(robot.framePose(robot.tool()), m_target);
}
```

- `robot.inverseKinematics(frame, pose)` returns one angle per joint, or
  nothing (`std::nullopt`) if the pose cannot be reached.
- Joints also have plain ids (`JointId`, `0 … size()-1`) with `double`
  values. Use them for loops; use typed handles when you name one joint.

The scenario uses a **deadline** assertion:

```yaml
    - { condition: "reach.error < 0.02", within: 5.0 }
```

In the simulator, this assertion is red at the start (the error is 3 cm), then
turns green when the tool arrives. A red assertion does not stop the run.

**Try it:** ask for 3 metres instead of 3 cm (`+= 3.0`). IK has no answer,
`status()` returns `FAILURE`, and the run ends at once:

```text
[FAIL] reach.error < 0.02  (reach.error = 3, not true within 5.000000 s)
[FAIL] default                  seed=1      controller failed at 0.01 s  time=0.01
```

---

## 5. Lesson 4 — a skill and a behavior tree

Lessons 1–3 drive the robot directly from `Mission::step`. This is simple,
but it does not scale: who decides the order? what if the arm is busy? what
if a sensor fails? Robotik answers with **skills** and **behavior trees**.

A skill is a small class with a `tick` method. `src/WaveSkill.cpp`:

```cpp
robotik::Status WaveSkill::tick(robotik::RobotContext& p_context, Seconds /*p_dt*/)
{
    robotik::JointSet& joints = p_context.robot.joints();
    robotik::JointId const base = joints.find("joint1");
    if (base == robotik::NO_JOINT)
    {
        return robotik::Status::FAILURE;
    }

    double const target = (m_waves % 2 == 0) ? 0.5 : -0.5;
    joints.moveTo(base, target);
    if (joints.reached(base, 0.02))
    {
        ++m_waves;
    }
    return m_waves >= 4 ? robotik::Status::SUCCESS : robotik::Status::RUNNING;
}

void WaveSkill::cancel(robotik::RobotContext& p_context)
{
    p_context.robot.joints().hold();   // cancelled or preempted: stop where we are
}
```

The mission registers it, with a **name** and the **resources** it needs:

```cpp
void SkillsMission::setup(robotik::Cell& p_cell, robotik::SceneView* /*p_view*/)
{
    robotik::SkillDescription wave;
    wave.name = "Wave";
    wave.resources = { p_cell.robot().resources().require("arm") };
    wave.priority = 100;
    p_cell.skills().add<WaveSkill>(std::move(wave));
}
```

Every registered skill becomes a behavior tree **Action** with the same
name. `scenarios/skills.bt.yaml`:

```yaml
BehaviorTree:
  Sequence:
    name: HelloSkills
    children:
      - Action:
          name: Home        # built-in skill
      - Action:
          name: Wave        # our skill
      - Action:
          name: Home
```

And the scenario uses it as its controller:

```yaml
  controller:
    behavior_tree: skills.bt.yaml
  assertions:
    - controller.success
    - skill_succeeded(Wave)
    - name: home before waving
      sequence: [ "skill_succeeded(Home)", "skill_started(Wave)" ]
```

```bash
./build/linux-gcc-debug/bin/Robotik-Headless build/linux-gcc-debug/lib/robotik/plugins/HelloWorld/scenarios/skills.scenario.yaml --trace
```

```text
  0 s  Home  SUCCEEDED  (0 s)
  0.01 s  Wave  SUCCEEDED  (4 s)
  4.02 s  Home  SUCCEEDED  (0.63 s)
```

Skills, resources, priorities, faults and trees are explained in
[05-Skills-and-Behavior-Trees.md](05-Skills-and-Behavior-Trees.md).

---

## 6. The plugin class

`src/Plugin.cpp` is the entry point. Here is the essential part:

```cpp
class HelloWorldPlugin final: public robotik::Plugin
{
public:

    // The id must match plugin.id in plugin.yaml.
    static robotik::PluginInfo describe()
    {
        return { .id = "robotik.hello_world" };
    }

    // Called before the simulation exists: create the mission and give it
    // to the host, which passes it to the simulation.
    RobotikPluginStatus setup(robotik::Host& p_host, std::string_view p_scenario) override
    {
        if (p_scenario == "hello")
        {
            m_mission = std::make_unique<HelloMission>();
        }
        else if (p_scenario == "wave")
        {
            m_mission = std::make_unique<WaveMission>();
        }
        // ... reach, skills ...
        else
        {
            p_host.error("unknown hello world scenario: " + std::string(p_scenario));
            return ROBOTIK_PLUGIN_ERR_SETUP;
        }

        p_host.bindMission(*m_mission);
        p_host.subscribe(ROBOTIK_EVENT_KEY_PRESSED);
        p_host.addMenuItem("Hello world", "Home", &onHome, this);
        p_host.addPanel("hello_world", "Hello world", &onDraw, this);
        m_host = p_host;
        return ROBOTIK_PLUGIN_OK;
    }

    // The host already destroyed the simulation: free the mission.
    RobotikPluginStatus shutdown(robotik::Host& /*p_host*/) override
    {
        m_mission.reset();
        return ROBOTIK_PLUGIN_OK;
    }

    // ...

private:

    std::unique_ptr<robotik::Mission> m_mission;
};

} // namespace

ROBOTIK_EXPORT_PLUGIN(HelloWorldPlugin)     // exports the C functions the host looks for
```

`setup` receives the **id of the scenario** (the `id` in `plugin.yaml`). One
plugin, one library, many scenarios: `setup` chooses the mission.

### Lifecycle

```text
plugin.yaml read          the host checks api_version, packs, dependencies
      │
      ▼
dlopen(.so)               only when a scenario of this plugin is chosen
      │
      ▼
setup(scenario id)        create the Mission, bindMission, menus, panels
      │
      ▼
Simulation built          from the world + scenario; Mission::setup, Mission::reset
      │
      ▼
start
      │
      ▼
loop: Cell::step → Mission::step → Plugin::update(dt)
      │
      ▼
stop                      the simulation still exists
      │
      ▼
shutdown                  the simulation is destroyed: free the mission
                          (the .so can stay loaded for the next scenario)
```

| `Plugin` method | When | Default |
|-----------------|------|---------|
| `setup(host, scenario)` | Prepare the mission, subscriptions, menu and panel | OK |
| `start(host)` | The run starts | OK |
| `update(host, dt)` | After each step (or *instead of* the step with `host.clock: plugin`) | OK |
| `onEvent(host, event)` | A subscribed event arrives | OK |
| `pause(host, paused)` | The user pressed Pause / Play | OK |
| `stop(host)` | End of the run; the simulation still exists | OK |
| `shutdown(host)` | Free the mission; the simulation is already destroyed | OK |

Override only what you need. Exceptions thrown by these methods are caught at
the plugin border and reported as errors: they do not crash the host.

### Host services

| `robotik::Host` method | Does |
|------------------------|------|
| `bindMission(mission)` | Gives the mission to the simulation that will be built |
| `simulation()` | The running `Simulation*`, or null before it exists |
| `log(level, text)` / `error(text)` | Messages (`ROBOTIK_LOG_INFO`, `_WARNING`, `_ERROR`) |
| `subscribe(kind)` | Receive one kind of event in `onEvent` |
| `addMenuItem(menu, item, callback, user)` | A menu entry in the simulator |
| `addPanel(id, title, draw, user)` | A panel in the simulator |
| `removeUi(handle)` | Remove a menu entry or a panel |
| `scenarioPath()` | The path of the scenario file |

Events (`RobotikEvent::kind`): `ROBOTIK_EVENT_KEY_PRESSED` (`code` = key),
`_SCENARIO_STARTED`, `_SCENARIO_STOPPED`, `_SKILL_COMPLETED`, `_SKILL_FAILED`
(`name` = skill), `_ASSERTION_CHANGED` (`name` = assertion, `flag` = passes).

### Menus and panels

The host calls plain C functions (the border between the host and the
plugin is C, so that compilers and versions can differ). The pattern is
always: a `static` function that casts `user` back to your object.

```cpp
    static void onHome(void* p_user)
    {
        static_cast<HelloWorldPlugin*>(p_user)->home();
    }

    static void onDraw(void* p_user, RobotikCanvas const* p_canvas)
    {
        static_cast<HelloWorldPlugin*>(p_user)->draw(robotik::Canvas(p_canvas));
    }

    void draw(robotik::Canvas const& p_canvas) const
    {
        p_canvas.text("scenario " + m_scenario);
        p_canvas.text("updates=" + std::to_string(m_updates) +
                      " keys=" + std::to_string(m_keys) +
                      " homes=" + std::to_string(m_homes));
    }
```

`Canvas` offers `text`, `textColored`, `button`, `checkbox`, `radio`,
`sliderInt` and `separator`. It is valid **only during the draw call**: never keep it.

The menu entry **Hello world → Home** (and the key **H**) asks the
simulation to run the built-in skill `Home`:

```cpp
    void home()
    {
        robotik::Simulation* simulation = m_host.simulation();
        if (simulation == nullptr)
        {
            return;
        }
        robotik::SkillId const id = simulation->skills().find("Home");
        if (id != robotik::NO_SKILL)
        {
            simulation->skills().request(id);
        }
    }
```

---

## 7. Tutorial: create your own plugin

Let us create `MyRobot`, from HelloWorld.

**Step 1. Copy.**

```bash
cp -r plugins/HelloWorld plugins/MyRobot
mv plugins/MyRobot/HelloWorld.mk plugins/MyRobot/MyRobot.mk
rm -rf plugins/MyRobot/experiments
```

**Step 2. Rename in four places.** The names must match, or the host refuses
the package.

| File | Change |
|------|--------|
| `plugins/MyRobot/MyRobot.mk` | the target `robotik_my_robot` (in `TARGETS`, its variables, and `$(call ROBOTIK_PLUGIN,robotik_my_robot,MyRobot)`), its `DESCRIPTION` |
| `plugins/MyRobot/plugin.yaml` | `plugin.id: robotik.my_robot`, `plugin.name`, `library.entrypoint: librobotik_my_robot.so`, the scenario list |
| `plugins/MyRobot/src/Plugin.cpp` | `describe()` returns `{ .id = "robotik.my_robot" }`; rename the class |
| `plugins/MyRobot/scenarios/*.scenario.yaml` | Keep only what you need; the `id` of each file must match an entry of `plugin.yaml` |

The rules:

```text
MyRobot.mk  TARGETS += robotik_my_robot
                 │
                 └──► produces librobotik_my_robot.so
                                    │
plugin.yaml  library.entrypoint: librobotik_my_robot.so      (same file name)
plugin.yaml  plugin.id: robotik.my_robot
                                    │
Plugin.cpp   describe() { .id = "robotik.my_robot" }          (same id)
plugin.yaml  scenarios.entries[].id: patrol
                                    │
Plugin.cpp   setup(host, "patrol")                            (same scenario id)
```

**Step 3. Keep one scenario and one mission.** For example, delete the
lessons you do not need from `src/`. `MyRobot.mk` compiles every `src/*.cpp`; to list them instead:

```make
robotik_my_robot.SOURCES := src/Plugin.cpp src/PatrolMission.cpp
```

**Step 4. Build and run.**

```bash
make robotik_my_robot
./build/linux-gcc-debug/bin/Robotik-Headless build/linux-gcc-debug/lib/robotik/plugins/MyRobot/scenarios/patrol.scenario.yaml
./build/linux-gcc-debug/bin/Robotik-Simulator          # Demos → My robot → ...
```

Add `MyRobot` to `PLUGINS` in the root `Makefile` (or `make PLUGINS+=MyRobot`),
so that its `MyRobot.mk` is read. `make` compiles the `.so` and copies `plugin.yaml`, the
library, and the folders `worlds/`, `scenarios/`, `experiments/` and `data/`
into `build/linux-gcc-debug/lib/robotik/plugins/MyRobot/`.

**Step 5. Use another robot.** Change the `model` of the robot in your world
file. Available models are listed in the `pack.yaml` files of `data/packs/`:

```bash
cat data/packs/*/pack.yaml
```

### Common errors

| Message | Cause |
|---------|-------|
| `scenario is not in a plugin package` | The file is not listed in `plugin.yaml`, or you ran a file outside `build/linux-gcc-debug/lib/robotik/plugins/` and `plugins/` |
| `unknown ... scenario: X` | `setup` does not know the scenario id `X` |
| `plugin id does not match plugin.yaml` | `describe().id` differs from `plugin.id` |
| `built for plugin API 99 ...` | `api_version` in `plugin.yaml` differs from the host ABI (`1`) |
| `plugin library not found: ...` | `library.entrypoint` differs from the `.so` built by the target (`lib` + its name + `.so`), or the plugin was not built |
| a pack `robotik.xxx` is named in the error | A pack of `resources.packs` is not found, or has an incompatible version |
| `Unknown Pinocchio frame: tcp` | An assertion uses `robot.tcp` but no tool is mounted: use `robot.tool0` |
| a `body` falls forever | The world has no `floor:` |

---

## 8. Asset packs: sharing 3D models

A robot, tool or object model is not copied into each plugin. It lives in a
**pack**: a folder with a `pack.yaml`, with a version.

```yaml
# data/packs/robotik.tools/pack.yaml
id: robotik.tools
version: 1.0.0
license: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
source: https://github.com/Lecrapouille/RobotIK
models:
  vacuum: tool_vacuum.urdf
  gripper: tool_gripper.urdf
```

A YAML file names a model `<pack id>.<key>`: `robotik.tools.vacuum`,
`robotik.robots.robot_6axis`. `AssetResolver` finds the real file; a plugin
never writes a path. The longest pack id wins: `robotik.robots.abb.irb2400`
is in the pack `robotik.robots.abb`.

| Level | Where | Example |
|-------|-------|---------|
| Shared pack | `data/packs/<id>/` | `robotik.robots`, `robotik.tools`, `robotik.robots.abb` |
| Pack of a plugin | `plugins/<Name>/data/` (`resources.directory: data`) | `robotik.painter` (the easel) |

Search order: `$ROBOTIK_DATA`, `data/packs` next to the executable folder,
`share/robotik/packs` of the install prefix, then the pack of each plugin.
The first one wins; another pack with the same id and another version is
reported as a conflict.

`license` is an SPDX expression. `NOASSERTION` means nobody checked the
license yet: this is the case of the ABB meshes. Check before you
redistribute them. The Assets panel of the simulator shows every pack with
its license.

---

## 9. `plugin.yaml` reference

```yaml
plugin:
  id: robotik.painter              # stable id; describe() returns the same
  name: Robotic painter            # shown in the Demos menu
  version: 1.0.0                   # major.minor.patch
  api_version: 1                   # C ABI version the library was built for
  category: Manipulation
  description: A robot arm paints colored strokes on an easel.   # optional

library:
  entrypoint: librobotik_painter.so

resources:
  directory: data                  # the asset pack of this plugin (optional)
  packs:                           # packs needed at run time
    - { id: robotik.robots, version: "1.0" }
    - { id: robotik.tools, version: "1.0" }

dependencies:
  plugins: []                      # other plugins that must be in the catalog
  system: []                       # system libraries, with license (see below)
  thirdparties: []                 # cloned libraries, with license (see below)

host:
  graphics: false                  # true: simulator only, Headless refuses it
  clock: simulation                # plugin: update() advances time itself
  view: { eye: [1.6, 1.2, 1.6], target: [0.35, 0.15, 0.0] }   # simulator camera

scenarios:
  directory: scenarios
  entries:
    - { id: painter, name: Robotic painter, file: painter.scenario.yaml }
    # optional: a submenu of the Demos menu, and a clock for this scenario only
    # - { id: painter_rl, name: Painter RL, file: painter_rl.rl.yaml, menu: Learning, clock: plugin }

experiments:
  directory: experiments
  entries:
    - { id: painter_experiment, name: Painting benchmark, file: painter.experiment.yaml }
```

| Key | Notes |
|-----|-------|
| `library.entrypoint` | A file name ending in `.so`, with no folder and no `..` |
| `resources.packs` | A missing pack, or one with an incompatible version: the scenario is refused with the reason. Version `"1.2"` accepts 1.2.0 and any newer 1.x, not 2.0 |
| `host.clock: plugin` | The host does not call `Simulation::step`: `update(dt)` advances the simulation. Default for every scenario; an entry can change it with its own `clock:` (the RL scenario of PickAndPlace) |
| `entries[].menu` | Groups the scenario in a submenu of the **Demos** menu (PickAndPlace: Behavior tree, Reinforcement learning) |
| `scenarios.entries` | Without `entries`, every runnable file of the folder is taken: `*.scenario.yaml`, `*.rl.yaml`, `*.experiment.yaml`, and an older YAML file without a role. Trees, domains and trajectories are skipped. The id is the name without the double extension |

### Third-party libraries of a plugin

A library that only one plugin uses is declared in the `<Name>.mk` of the
plugin, as a target of [MyMakefile](https://github.com/Lecrapouille/MyMakefile):
fetched on the first build, at the commit locked in `external/versions.lock`,
and built once in the cache of MyMakefile. `dependencies:` of `plugin.yaml`
lists them with their license, for the credits of the Plugin panel of the
simulator:

```yaml
dependencies:
  system:                          # installed on the system: pkg-config names
    - { name: opencv4, license: Apache-2.0 }
  thirdparties:                    # declared in <Name>.mk
    - { repo: AprilRobotics/apriltag, license: BSD-2-Clause, description: AprilTag fiducial detection }
    - { name: onnxruntime, url: "https://github.com/microsoft/onnxruntime", license: MIT }
```

| Field | Notes |
|-------|-------|
| `repo` | GitHub `user/repo[@ref]` |
| `name`, `url` | A library without a repository (a release of binaries) |
| `license` | SPDX name. Required: without it the library shows "not checked" |
| `description` | Optional. Put a value with a comma between quotes |

`plugins/LineFollower/LineFollower.mk` builds AprilTag with its CMake, and is
not built when OpenCV is not installed:

```make
ifeq ($(call robotik-missing,opencv4),)
    TARGETS += apriltag robotik_line_follower
    apriltag.GIT := https://github.com/AprilRobotics/apriltag.git
    apriltag.TYPE := cmake
    apriltag.PUBLIC_INCLUDES := include/apriltag
    apriltag.LIBRARIES := apriltag

    robotik_line_follower.SOURCES := src/*.cpp
    robotik_line_follower.DEPENDS := apriltag
    robotik_line_follower.PKG_CONFIG := opencv4
    $(eval $(call ROBOTIK_PLUGIN,robotik_line_follower,LineFollower))
else
    $(call print-warning,LineFollower skipped: ...)
endif
```

A release of binaries is a target with a `URL` (ONNX Runtime:
`$(eval $(ROBOTIK_ONNXRUNTIME))` of `plugins/plugin.mk`); data made by a
script is a Make rule the plugin target depends on (`MicroDuck.mk`).

---

## 10. Where the hosts look for plugins

Nothing depends on the current folder. Both hosts search, in this order:

1. the path given to `scan`, if any;
2. each folder of `$ROBOTIK_PLUGINS` (separated by `:`);
3. `plugins/` next to the executable;
4. `../lib/robotik/plugins` from the executable: `build/linux-gcc-debug/lib/robotik/plugins` in the build tree, `lib/robotik/plugins` of the install prefix.

A scenario given by its path brings its package with it:
`Robotik-Headless plugins/Painter/scenarios/painter.scenario.yaml` works from the
sources; the library is searched next to the manifest, then next to the
executable.

---

## 11. Good rules for a plugin

- **The plugin does not own the world.** The `Simulation` builds the entities
  from the world file and destroys them. Find them by id
  (`simulation.entity("canvas")`, `simulation.pose("canvas")`); never keep a
  pointer to a world object after `shutdown`.
- **A reset rebuilds the state** without unloading the plugin: put all
  episode state back in `Mission::reset`.
- **Skills see beliefs, not truth.** In a skill, use `RobotContext`
  (`robot`, `world_model`). Ground truth (`Cell::pose`) is for metrics
  and assertions.
- **Publish metrics, do not judge.** The scenario file holds the verdict.
- **Units are SI**: metres, radians, seconds. Use the typed handles
  (`Revolute`, `Prismatic`, `Seconds`, `Radians`) when you can.
- **Headless first.** `view` is null and there is no camera image in
  headless: your mission must still work (see `perception.blind`).

Next: [05-Skills-and-Behavior-Trees.md](05-Skills-and-Behavior-Trees.md).
