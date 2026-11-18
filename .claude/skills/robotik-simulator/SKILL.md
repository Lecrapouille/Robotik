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
| Workspace.cpp | the 4 workspaces, their dock layouts, menu bar, tab alerts (red), run bar (play, clock, seed, emergency stop), status bar (links), shortcuts |
| Ui.hpp | declaration of every panel and shared widget; read it first |
| Widgets.cpp | `badge`, `helpMarker`, `emptyState`, `beginTable`, `textView`, `rangeBar` |
| ScenePanel.cpp | main area of a workspace (`mainPanel`: view selector, Full view), 3D view, click to select (`Scene::pick`), boxes of the selected object (truth, belief), run frame color (`runColor`), emergency stop (`toggleEmergency`) |
| PackagePanels.cpp / OverviewPanels.cpp | Plugin workspace: Plugin, Files (tabs), View, Assets, Assertions |
| RobotPanel.cpp / TeachPanel.cpp | Joints (+ plots), Teach Pendant |
| MissionPanels.cpp / SkillsPanel.cpp / PlannerPanel.cpp | Behavior tree, Resources, Skills, Planner |
| InspectorPanel.cpp | Inspector, one per workspace (skill, resource, robot, object, overview), `selectItem` / `isSelected` / `itemLink` / `showInspector` |
| UiScreenshot.cpp | `--screenshot-ui`: PNG of the whole window (check a change without a person: `--at`, `--workspace`, `--select`) |
| GraphView.cpp | Execution graph (a view of the main area only): plan steps with their skills and what they wait for, or the behavior tree laid out left to right |
| TimelinePanel.cpp | Timeline (time window, zoom, rows by robot, resource rows); `sampleTimeline` records faults and tree failures every frame |
| AutonomyPanels.cpp / CameraPanel.cpp | World model (objects + symbolic beliefs), Perception, Cameras, the camera inset of the scene (`cameraInset`) |
| PluginUi.cpp | Demos menu, plugin menus and panels (`RobotikCanvas`) |

## Rules

- A panel is a free function `void xxxPanel(App&)` declared in `Ui.hpp`; its
  content without the window is `xxxView(App&)` when another panel reuses
  it (e.g. `plannerView` in Mission and in the Plugin View panel), or when
  it can fill the main area or the work area (`ViewKind`: `drawView`,
  `mainPanel` and its selector, `toggleMaximized`, `maximizedWindow`). Such
  a panel draws `viewNote` instead of its view when `viewElsewhere` says it
  is shown elsewhere, and sets `UiState::hovered_view` when hovered. A new
  view: add it to `ViewKind`, `drawView`, `viewName` and `MAIN_VIEWS`.
- Panel state lives in `UiState` (App.hpp), never in a `static` that outlives
  a scenario load. Reset it in `App::load` (and `App::reset` if it depends
  on the run).
- A new or renamed panel: dock it in `layout()` (Workspace.cpp), call it in
  `drawPanels`, and **bump `LAYOUT_VERSION`** so saved layouts are rebuilt.
  A workspace `anchor` must name an existing window.
- One selection for every panel (`UiState::selection`: robot, skill,
  resource or object, by name). A panel that names one of them draws it with
  `itemLink` so a click shows it in the Inspector. Selecting never acts.
- Check a visual change with `--screenshot-ui x.png --at <s>` (and
  `--select`), then read the PNG; do not capture the desktop.
- Status colors only from `Colors.hpp`: GREY idle, ORANGE running/held,
  GREEN success, RED failure/fault, BLUE info/paused/shared.
- Never write the truth into what skills see; panels may read the truth
  (`Cell::pose`) to compare with beliefs.
- Manual mode (`App::teach.manual`): the mission is suspended and the robot
  steps even when paused. Any manual command calls `takeArm(App&)` first.
- The simulator knows no demo: no object id, tool kind or joint name of a
  plugin in a panel (see "Core stays agnostic" in robotik-plugin). A panel
  for one demo is a plugin panel (`addPanel`).
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
