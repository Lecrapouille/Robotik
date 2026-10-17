---
name: robotik-simulator
description: Change the RobotIK simulator interface (simulator/, Dear ImGui): add or move a panel, a workspace layout, a Files tab, a status color, or work toward editing scenarios from the interface. Use for any change under simulator/*.cpp|hpp.
---

# RobotIK simulator (ImGui host)

`simulator/` is a host, not the library: it reads `robotik::Simulation`, it
never adds robot logic. Everything must also work headless, so a feature the
operator needs in a run (faults, metrics, verdicts) belongs in the core or in
scenario YAML, and the simulator only shows or triggers it.

## Map

| File | Holds |
|------|-------|
| App.hpp / App.cpp | `App` (load, reset, step loop, Manual mode), `UiState` (all panel state), `FileTab` |
| Workspace.cpp | the 4 workspaces, their dock layouts, menu bar, tab alerts (red), status bar, shortcuts |
| Ui.hpp | declaration of every panel and shared widget; read it first |
| Widgets.cpp | `badge`, `helpMarker`, `emptyState`, `beginTable`, `textView`, `rangeBar` |
| ScenePanel.cpp | 3D view, run controls, run frame color (`runColor`), emergency stop |
| PackagePanels.cpp / OverviewPanels.cpp | Plugin workspace: Plugin, Files (tabs), View, Assets, Assertions |
| RobotPanel.cpp / TeachPanel.cpp | Joints (+ plots), Teach Pendant |
| MissionPanels.cpp / SkillsPanel.cpp / PlannerPanel.cpp | Behavior tree, Timeline, Resources, Skills, Planner |
| AutonomyPanels.cpp / CameraPanel.cpp | World model (objects + symbolic beliefs), Perception, Cameras |
| PluginUi.cpp | Demos menu, plugin menus and panels (`RobotikCanvas`) |

## Rules

- A panel is a free function `void xxxPanel(App&)` declared in `Ui.hpp`; its
  content without the window is `xxxView(App&)` when another panel reuses
  it (e.g. `plannerView` in Mission and in the Plugin View panel).
- Panel state lives in `UiState` (App.hpp), never in a `static` that outlives
  a scenario load. Reset it in `App::load` (and `App::reset` if it depends
  on the run).
- A new or renamed panel: dock it in `layout()` (Workspace.cpp), call it in
  `drawPanels`, and **bump `LAYOUT_VERSION`** so saved layouts are rebuilt.
  A workspace `anchor` must name an existing window.
- Status colors only from `Colors.hpp`: GREY idle, ORANGE running/held,
  GREEN success, RED failure/fault, BLUE info/paused/shared.
- Never write the truth into what skills see; panels may read the truth
  (`Simulation::pose`) to compare with beliefs.
- Manual mode (`App::teach.manual`): the mission is suspended and the robot
  steps even when paused. Any manual command calls `takeArm(App&)` first.
- Update doc/02-Getting-Started.md (workspace table) when a panel changes.

## Toward a scenario editor (planned, not built)

Goal: create and edit worlds and scenarios from the interface. Keep these
hooks when you touch the Files panel or the views:

1. **Model**: edit `robotik::ScenarioSource` (Scenario.hpp): it already
   keeps the scenario and world as editable `ConfigNode` trees and has
   `override("world.entities.<id>.<key>", value)`. Missing: a YAML writer
   for `ConfigNode` (rapidyaml can emit) that keeps comments where possible.
2. **Validate** without running: `Scenario::read(source, assets)` throws with
   a message; show it in the Plugin tab alert (`alertOf` in Workspace.cpp).
3. **Views become forms**: World view (entity tree) → add/remove entity, pose
   gizmo in the 3D view; Scenario view → assertions, faults (reuse the
   Resources fault form), controller (tree file or planner goal).
4. **Files stay text**: `textView` is read-only today; an editable text tab
   must re-validate on save and reload with `requestScenario`.
5. **Package rules still apply**: a new scenario must be added to
   `plugin.yaml` `scenarios.entries` and use package-relative paths
   (PackagePath.hpp), with the role suffix (`*.scenario.yaml`,
   `*.world.yaml`, see robotik-scenario).
6. **LLM**: the Planner view keeps the goal on top: a sentence → PDDL goal
   step goes there, before the planner.
