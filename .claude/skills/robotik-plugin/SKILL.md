---
name: robotik-plugin
description: Create, rename or extend a RobotIK plugin (plugins/<Name>/ with plugin.yaml, Makefile, src/Plugin.cpp, missions). Use when adding a demo, a scenario to a plugin, a menu/panel/key binding, or debugging "scenario is not in a plugin package", id mismatch, library not found.
---

# RobotIK plugin

Template to copy: `plugins/HelloWorld/` (4 lessons: Mission only, joints/FK,
IK, skill + behavior tree; panel, menu, key 'H'). Reference demo with
perception + BT: `plugins/PickAndPlace/`.

## New plugin checklist

1. `cp -r plugins/HelloWorld plugins/<Name>`; drop unused `src/` files,
   `experiments/`, scenarios.
2. Keep these names consistent (otherwise the host refuses the package):
   - `Makefile`: `TARGET_NAME := robotik_<snake>` → builds `librobotik_<snake>.so`;
     `PLUGIN_NAME := <Name>`; `PLUGIN_DIR := $(P)/plugins/<Name>`;
     `LIB_FILES := $(PLUGIN_SRC)/Plugin.cpp ...` (every .cpp).
   - `plugin.yaml`: `plugin.id: robotik.<snake>`, `api_version: 1`,
     `library.entrypoint: librobotik_<snake>.so`, `resources.packs` used by
     the worlds, `scenarios.entries[] {id, name, file}`.
   - `src/Plugin.cpp`: `describe()` returns `{ .id = "robotik.<snake>" }`;
     `setup(host, scenario)` handles every scenario `id`, else
     `host.error(...)` + `return ROBOTIK_PLUGIN_ERR_SETUP`.
   - each `scenarios/<s>.scenario.yaml`: `scenario.id` = the entry id.
3. One `.cpp` ends with `ROBOTIK_EXPORT_PLUGIN(<Class>)`.
4. Every new file starts with the SPDX header (copy from HelloWorld).
5. Build + run:
   ```bash
   make -C plugins/<Name> -j"$(nproc)"
   ./build/Robotik-Headless build/plugins/<Name>/scenarios/<s>.scenario.yaml --trace
   ```
   No list to update: the root Makefile builds every `plugins/*/Makefile`.

## Skeleton

```cpp
class MyMission final: public robotik::Mission
{
public:
    void setup(robotik::Simulation& p_sim, robotik::SceneView* p_view) override; // skills, detectors; p_view null headless
    void reset(robotik::Simulation& p_sim, robotik::Seed p_seed) override;      // clear episode state
    void step(robotik::Simulation& p_sim, Seconds p_dt) override;               // after each robot step
    void measure(robotik::Simulation const& p_sim, robotik::Metrics& p_m) const override; // p_m.set("my.x", v)
    [[nodiscard]] robotik::Status status(robotik::Simulation const& p_sim) const override; // only used without BT
};

class MyPlugin final: public robotik::Plugin
{
public:
    static robotik::PluginInfo describe() { return { .id = "robotik.my" }; }
    RobotikPluginStatus setup(robotik::Host& p_host, std::string_view p_scenario) override
    {
        if (p_scenario != "main") { p_host.error("unknown scenario"); return ROBOTIK_PLUGIN_ERR_SETUP; }
        m_mission = std::make_unique<MyMission>();
        p_host.bindMission(*m_mission);              // mandatory to drive the simulation
        return ROBOTIK_PLUGIN_OK;
    }
    RobotikPluginStatus shutdown(robotik::Host&) override { m_mission.reset(); return ROBOTIK_PLUGIN_OK; }
private:
    std::unique_ptr<MyMission> m_mission;
};
ROBOTIK_EXPORT_PLUGIN(MyPlugin)
```

Lifecycle: manifest read → dlopen on scenario choice → `setup` (no
simulation yet) → Simulation built (`Mission::setup`, `reset`) → `start` →
[`Simulation::step` → `Mission::step` → `Plugin::update`]* → `stop` →
`shutdown` (simulation already destroyed).

Host services: `bindMission`, `simulation()` (null before attach), `log`,
`error`, `subscribe(ROBOTIK_EVENT_KEY_PRESSED | _SCENARIO_STARTED |
_SCENARIO_STOPPED | _SKILL_COMPLETED | _SKILL_FAILED | _ASSERTION_CHANGED)`,
`addMenuItem(menu, item, void(*)(void*), this)`,
`addPanel(id, title, void(*)(void*, RobotikCanvas const*), this)`.
Callbacks are static functions casting `user` back; wrap the canvas as
`robotik::Canvas(p_canvas)` (`text`, `textColored`, `button`, `checkbox`,
`radio`, `sliderInt`, `separator`), valid only during the draw.

## Rules

- Do not judge in C++: publish metrics, assert in YAML. Prefix metric names.
- Do not keep pointers to world entities across resets/shutdown; use
  `simulation.entity("id")`, `simulation.pose("id")`.
- Must run headless (no view, no camera image → `perception: {oracle: when_blind}`).
- `host.clock: plugin` (or `clock: plugin` on one scenario entry) → `update(dt)` must advance the simulation itself.
- Models are asset ids (`robotik.robots.robot_6axis`), never paths; a
  plugin-local pack goes in `plugins/<Name>/data/pack.yaml` +
  `resources.directory: data`.

## Core stays agnostic of the demos

The demo plugins exist to prove that RobotIK fits **any** robot project
(arm, humanoid, fly, mobile robot, hexapod...). So the core (`src/`,
`include/`) and the hosts (`simulator/`, `headless/`) must not know a demo:

- no demo name, object id, joint name, tool kind or task in core code
  (`red_cube`, `drill_spindle_joint`, "painting"...);
- a feature one demo needs goes in its plugin; it goes in the core only as
  a generic concept that a second, different project could use too;
- a core concept keeps a generic name (`ecs::Shape`, not `SceneObject` of a
  pick and place).

When you meet a leak, fix it or list it below and tell the user.

Known leaks (October 2026), to fix:

| Where | Leak | Generic direction |
|-------|------|-------------------|
| `Systems/AbstractGrasp.cpp`, `Simulation.cpp` `trackPropPenetration` | the abstract level puts a released box at the centre of the container below (vacuum only) | done for `rigid` (`RigidGrasp`); a gripper interface for fingers (`contact`, FIDELITY.md) |
| `SimulationTruth.cpp` `holding(g)` | only for `VacuumGripper` | a gripper interface (`holding`, `held`) for any grasping actuator |
| `Backends/SceneView.hpp` `openCanvas`, `ink` | the Painter sheet in the backend interface | a generic dynamic texture on an entity |
| `Simulation.cpp` built-in `Home` | needs an arm joint group; no sense for a humanoid or a fly | `Home` only when the robot declares a `home:` pose |
| `simulator/TeachPanel.cpp` `triggerTool` | knows vacuum, gripper, drill and `drill_spindle_joint` | each tool declares its trigger (actuator) in its pack |
| `ecs::Shape` | box and container only | sphere, cylinder, mesh |
| `Perception/WorldModel.cpp` monocular estimate | objects on a known top plane (tabletop) | a depth source or a declared support |

## Errors

| Message | Fix |
|---------|-----|
| `scenario is not in a plugin package` | list the file in `plugin.yaml` |
| `plugin id does not match plugin.yaml` | `describe().id` = `plugin.id` |
| `plugin library not found: ...` | `entrypoint` = `lib` + `TARGET_NAME` + `.so`; build it |
| `built for plugin API N` | `api_version: 1` |
| `Unknown Pinocchio frame: tcp` | no tool mounted: use `robot.tool0` |

Tests touching plugins: `tests/Plugin/TestPlugin.cpp` (HelloWorld ids,
panel text `updates=N keys=N homes=N`, scenario names). Update them if you
change HelloWorld.
