# 🔌 Plugin API

[⏫ API reference](README.md)

Header: `RobotIK/Package/Plugin/PluginAPI.hpp` (C++ helper over the C border `RobotIK/Package/Plugin/PluginABI.h`).

## 🔌 `robotik::ScenarioPlugin`

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

## 🔌 `robotik::Plugin`

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

## 🏠 `robotik::Host`

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

## 📨 `RobotIKEvent`

| `kind` | Fields |
|--------|--------|
| `ROBOTIK_EVENT_KEY_PRESSED` | `code`: the key (`'H'`) |
| `ROBOTIK_EVENT_SCENARIO_STARTED`, `_STOPPED` | — |
| `ROBOTIK_EVENT_SKILL_COMPLETED`, `_SKILL_FAILED` | `name`: the skill |
| `ROBOTIK_EVENT_ASSERTION_CHANGED` | `name`: the assertion, `flag`: 1 if it passes |

## 🖌 `robotik::Canvas`

Valid only during the draw callback.

`text(line)`, `textColored(r, g, b, line)`, `bool button(label)`, `bool checkbox(label, bool&)`, `bool radio(label, bool active)`, `bool sliderInt(label, int&, min, max)`, `separator()`. The `bool` results are true when the user acted.
