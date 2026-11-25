// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file Ui.hpp
/// Editors of the operator interface and their layout presets.
///
/// Each editor has one stable window identity across layout presets.
#pragma once

#include "App.hpp"
#include "Colors.hpp"

#include "RobotIK/Task/Scheduler.hpp"
#include "RobotIK/Task/Skills/SkillNodes.hpp"

#include <imgui.h>

#include <cstdint>
#include <string>

// --- Shared helpers (Widgets.cpp) -------------------------------------------

/// Stable editor title; the legacy layout key does not change its identity.
std::string panelTitle(char const* p_name, char const* p_key);

/// Rounded label with a colored background (a status badge).
void badge(char const* p_text, ImVec4 const& p_color);

/// Small grey "(?)" with a tooltip.
void helpMarker(char const* p_text);

/// Text shown when a panel has nothing to display.
void emptyState(char const* p_text);

/// Small pictures drawn with lines (the simulator has no icon font).
enum class Icon : std::uint8_t
{
    Expand,  ///< Four corners pointing out: show over the whole work area.
    Restore, ///< Four corners pointing in: back to the layout.
    Close,   ///< A cross: hide.
};
/// A square of side @p_size at @p_min with @p_icon in it, readable over a
/// 3D view; lighter when @p_hovered.
void drawIconBox(ImDrawList* p_draw,
                 ImVec2 p_min,
                 float p_size,
                 Icon p_icon,
                 bool p_hovered);
/// A square button showing @p_icon, readable over a 3D view; true when
/// clicked. @p_tooltip may be null.
bool iconButton(char const* p_id, Icon p_icon, char const* p_tooltip);

/// ImGui::BeginTable with a smaller font and tighter cells, so that
/// columns are not cut. Call endTable() only when it returns true.
bool beginTable(char const* p_id,
                int p_columns,
                ImGuiTableFlags p_flags = 0,
                ImVec2 const& p_size = ImVec2(0.0F, 0.0F));
void endTable();

/// The monospace font (Cousine), or null when it is not loaded.
ImFont* monoFont();

/// Read-only view of a text file: line numbers, monospace font,
/// a button to copy the path and one to copy the text. YAML and PDDL
/// are coloured by syntax.
void textView(std::filesystem::path const& p_path, std::string const& p_text);
/// Text of @p_path, or a short note for a binary or a large file.
std::string readShown(std::filesystem::path const& p_path);

/// Horizontal bar from @p_lower to @p_upper with @p_value filled,
/// an optional target tick, and a centred label.
void rangeBar(double p_value,
              double p_lower,
              double p_upper,
              double p_target,
              bool p_has_target,
              char const* p_label,
              ImVec4 const& p_color);

// --- Shell (Workspace.cpp) --------------------------------------------------

/// Switches to another preset and applies its default editors and positions.
void selectWorkspace(App& p_app, Workspace p_workspace);
/// Requests expansion of one editor; an empty name restores the layout.
void toggleMaximized(App& p_app, std::string const& p_window);
/// Applies pending expansion/restoration before ImGui::NewFrame().
void updatePanelLayout(App& p_app);
/// Name of a registered editor, independent of the selected layout preset.
char const* panelName(PanelKind p_panel);
/// Starts an editor with common close, expand and context-menu controls.
/// Always pair with ImGui::End().
bool beginEditor(App& p_app,
                 char const* p_title,
                 bool& p_open,
                 ImGuiWindowFlags p_flags = 0);

/// Registers window visibility settings before ImGui reads simulator.ini.
void initializePanelSettings(App& p_app);
/// Saves visibility while App is alive and removes its settings callback.
void shutdownPanelSettings(App& p_app);
/// Starts a closable optional window. Always pair with ImGui::End().
bool beginPanel(App& p_app,
                PanelKind p_panel,
                char const* p_title,
                ImGuiWindowFlags p_flags = 0);
/// Opens and focuses the existing editor without changing the layout preset.
void showPanel(App& p_app, PanelKind p_panel);
void drawPanels(App& p_app);

// --- Panels shared by several workspaces ------------------------------------

/// Independent editors for the scene and execution graph.
void scenePanel(App& p_app);
void graphPanel(App& p_app);
/// Contents of the Scene panel, without its window: in the panel, or over
/// the whole work area when maximized.
void sceneView(App& p_app, bool p_compact = false);
/// Small live 3D view while another editor fills the work area.
void sceneInset(App& p_app);
/// Color of the frame around the 3D view: orange while the run goes
/// on, green when it succeeded, red when it failed. False without a run, or
/// before it starts.
bool runColor(App const& p_app, ImVec4& p_color);
/// True when the scenario has a Stop skill (emergency stop).
bool emergencyAvailable(App const& p_app);
/// True while the Stop skill holds the robot.
bool emergencyEngaged(App const& p_app);
/// Engages the Stop skill, or releases it.
void toggleEmergency(App& p_app);
/// Every robot camera, with the detections drawn on (CameraPanel.cpp).
void camerasPanel(App& p_app, char const* p_key);
/// The camera of the inset, large, without a window (maximized).
void cameraView(App& p_app);
/// The cameras of the first robot (none without a scenario).
std::vector<robotik::Camera*> robotCameras(App& p_app);
/// One robot camera over the bottom-right corner of the 3D view drawn at
/// @p_view_at, with the detections; a button shows it again once hidden.
void cameraInset(App& p_app, ImVec2 p_view_at, ImVec2 p_view_size);
/// Assertions of the scenario, as a table (OverviewPanels.cpp).
void assertionsPanel(App& p_app, char const* p_key);
/// Assertions without their window.
void assertionsView(App& p_app);

// --- Plugin workspace (PackagePanels.cpp, OverviewPanels.cpp) ---------------

/// The loaded plugin: description, files, dependencies and the
/// scenario running.
void pluginPanel(App& p_app);
/// One panel for the view of the file being read: Scenario (its
/// settings and assertions), World, Behavior tree or Experiment.
void viewPanel(App& p_app);
/// Switches the view panel; with @p_editor, Files shows that tab too.
void showView(App& p_app, FileTab p_view, bool p_editor);
/// Contents of the views (no window of their own).
void scenarioView(App& p_app);
void worldView(App& p_app);
void experimentView(App& p_app);
/// Shows @p_path in the tab of its kind. A scenario of the package is
/// loaded, an experiment becomes the one of the Experiment panel.
void showFile(App& p_app, std::filesystem::path const& p_path);
/// Makes @p_path the experiment of the Experiment panel.
void selectExperiment(App& p_app,
                      std::filesystem::path const& p_path,
                      bool p_focus);
/// Every asset pack found: models, version, license, source. A click on a
/// model or a file of a pack shows it in the Asset tab of Files.
void assetsPanel(App& p_app);
/// Contents of the Asset tab of Files: the file clicked in Assets, as a 3D
/// model, a picture or a text.
void assetView(App& p_app);
/// Reads the file clicked in Assets in the previous frame. Called before
/// any panel, when no texture of the old file is drawn yet.
void openClickedAsset(App& p_app);
void filesPanel(App& p_app);

// --- Robot workspace (RobotPanel.cpp, TeachPanel.cpp) -----------------------

void robotPanel(App& p_app);
void plotsPanel(App& p_app);
/// Contents of the Joint plots panel, without its window.
void plotsView(App& p_app);
void teachPanel(App& p_app);
/// Draws X, Y, Z axes on the frame the Teach Pendant moves, or hides
/// them. Called every frame.
void syncTeachAxes(App& p_app, bool p_visible);
/// Takes the arm from the mission for manual moves.
void takeArm(App& p_app);

// --- Selection and Inspector (InspectorPanel.cpp) ----------------------------

/// Makes @p_name the selection of every panel.
void selectItem(App& p_app, SelectionKind p_kind, std::string const& p_name);
/// True when @p_name of @p_kind is the selection.
bool isSelected(App const& p_app,
                SelectionKind p_kind,
                std::string const& p_name);
/// A clickable name: it selects the item, and opens the Inspector when it
/// is closed. Returns true when clicked.
bool itemLink(App& p_app, SelectionKind p_kind, std::string const& p_name);
/// What is selected: a robot, a skill, a resource or an object, its state
/// and links to what it is related to. It can keep one item (Pin). One
/// shared window across layout presets.
void inspectorPanel(App& p_app, char const* p_key);
/// Opens and focuses the shared Inspector.
void showInspector(App& p_app);

// --- Mission workspace (MissionPanels.cpp, TimelinePanel.cpp, GraphView.cpp)

/// Root of the nested tree of a SubTree node, else null.
bt::Node const* subtreeRoot(bt::Node const& p_node);
/// The skill phases, faults and tree failures on one time axis that can
/// follow the present, be zoomed and moved; one row per skill, grouped by
/// robot, and one per resource.
void timelinePanel(App& p_app);
/// Contents of the Timeline panel, without its window.
void timelineView(App& p_app);
/// The execution graph (GraphView.cpp): the steps of the plan, what they
/// wait for and the skills they run; the behavior tree without a planner.
/// Contents of the Execution graph editor.
void graphView(App& p_app, bool p_preview = false);
// ****************************************************************************
/// What an event is about, for the filters.
// ****************************************************************************
enum class RunEventKind : std::uint8_t
{
    Skill,
    Resource,
    Plan,
    Verdict,
};

// ****************************************************************************
/// One thing that happened in the run (a line of the Events panel, a mark
/// of the Timeline).
// ****************************************************************************
struct RunEvent
{
    double time = 0.0;
    RunEventKind kind = RunEventKind::Skill;
    /// A failure, a fault, a wait on a resource: kept by "Problems only".
    bool problem = false;
    ImVec4 color;
    std::string text;
    /// The item the line is about, selected by a click; None: no item.
    SelectionKind item = SelectionKind::None;
    std::string name;
    /// The same line repeated in a row (a skill refused at each step):
    /// how many times, and the time of the last one.
    std::size_t count = 1;
    double until = 0.0;
};

/// What happened in the run, sorted by time (EventsPanel.cpp).
/// @param p_skills With the phases of the skills (many); without them, the
///   faults, plans, steps, tree failures and verdicts only.
std::vector<RunEvent> runEvents(App const& p_app, bool p_skills);

/// What happened in the run, one dated line each, with filters
/// (EventsPanel.cpp). A click on a time shows it in the Timeline.
void eventsPanel(App& p_app);
/// Contents of the Events panel, without its window.
void eventsView(App& p_app);
/// Records what the Timeline cannot read back later (resource faults, tree
/// failures). Called every frame, whatever the workspace shown.
void sampleTimeline(App& p_app);
/// Who holds each resource, and faults injected by hand.
void resourcesPanel(App& p_app);
/// The PDDL planner: goal, plan, beliefs, history (PlannerPanel.cpp).
void plannerPanel(App& p_app);
/// Contents of the Planner panel, without its window.
void plannerView(App& p_app);
/// Texts behind a plan: domain, problem, output, generated tree
/// (Files > Planner).
void plannerTexts(App& p_app);
/// Symbolic beliefs: the atoms the observers read now.
void beliefsView(App& p_app);

// --- Autonomy workspace (AutonomyPanels.cpp) --------------------------------

void worldModelPanel(App& p_app);
void perceptionPanel(App& p_app);

// --- Plugin UI (PluginUi.cpp) -----------------------------------------------

/// Loads @p_file at the start of the next frame, never in the middle
/// of one: menus and panels may still be reading the catalog and the old run.
void requestScenario(App& p_app, std::filesystem::path const& p_file);

/// The Tutorials menu (packages of category Tutorial) and the Demos menu
/// (the other packages), each with its packages and their scenarios.
void drawDemosMenu(App& p_app);
/// Menus added by the loaded plugin.
void drawPluginMenus(App& p_app);
/// Stable, namespaced title of a plugin editor, independent of its handle.
std::string pluginEditorKey(App const& p_app,
                            robotik::PluginPanel const& p_panel);
std::string pluginWindowTitle(App const& p_app,
                              robotik::PluginPanel const& p_panel);
/// Panels added by the loaded plugin. @p_dock, when not zero, docks
/// them there first; @p_focus brings the first one to the front.
void drawPluginPanels(App& p_app, ImGuiID p_dock, bool p_focus);
