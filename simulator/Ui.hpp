// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

//! @file Ui.hpp
//! @brief Panels of the operator interface, grouped by workspace.
//!
//! A panel shown in several workspaces is drawn under one window per
//! workspace: its title is "Name##workspace", so each layout keeps its own
//! place for it. All copies read the same App state.
#pragma once

#include "App.hpp"
#include "Colors.hpp"

#include "Robotik/Runtime/Scheduler.hpp"

#include <imgui.h>

#include <string>


// --- Shared helpers (Widgets.cpp) -------------------------------------------

//! @brief Window title of a panel in one workspace: "Name##key".
std::string panelTitle(char const* p_name, char const* p_key);

//! @brief Rounded label with a colored background (a status badge).
void badge(char const* p_text, ImVec4 const& p_color);

//! @brief Small grey "(?)" with a tooltip.
void helpMarker(char const* p_text);

//! @brief Text shown when a panel has nothing to display.
void emptyState(char const* p_text);

//! @brief ImGui::BeginTable with a smaller font and tighter cells, so that
//! columns are not cut. Call endTable() only when it returns true.
bool beginTable(char const* p_id, int p_columns, ImGuiTableFlags p_flags = 0,
                ImVec2 const& p_size = ImVec2(0.0f, 0.0f));
void endTable();

//! @brief The monospace font (Cousine), or null when it is not loaded.
ImFont* monoFont();

//! @brief Read-only view of a text file: line numbers, monospace font,
//! a button to copy the path and one to copy the text.
void textView(std::filesystem::path const& p_path, std::string const& p_text);

//! @brief Horizontal bar from @p_lower to @p_upper with @p_value filled,
//! an optional target tick, and a centred label.
void rangeBar(double p_value,
              double p_lower,
              double p_upper,
              double p_target,
              bool p_has_target,
              char const* p_label,
              ImVec4 const& p_color);

// --- Shell (Workspace.cpp) --------------------------------------------------

void drawPanels(App& p_app);

// --- Panels shared by several workspaces ------------------------------------

//! @brief The 3D view with the run controls on top (ScenePanel.cpp).
void scenePanel(App& p_app, char const* p_key);
//! @brief True while the Stop skill holds the robot.
bool emergencyEngaged(App const& p_app);
//! @brief Engages the Stop skill, or releases it.
void toggleEmergency(App& p_app);
//! @brief Every robot camera, with the detections drawn on (CameraPanel.cpp).
void camerasPanel(App& p_app, char const* p_key);
//! @brief Assertions of the scenario, as a table (OverviewPanels.cpp).
void assertionsPanel(App& p_app, char const* p_key);
//! @brief The behavior tree with the last status of each node
//! (MissionPanels.cpp).
void treePanel(App& p_app, char const* p_key);
//! @brief Contents of these two panels, without their window.
void treeView(App& p_app);
void assertionsView(App& p_app);

// --- Plugin workspace (PackagePanels.cpp, OverviewPanels.cpp) ---------------

//! @brief The loaded plugin: description, files, dependencies and the
//! scenario running.
void pluginPanel(App& p_app);
//! @brief One panel for the view of the file being read: Scenario (its
//! settings and assertions), World, Behavior tree or Experiment.
void viewPanel(App& p_app);
//! @brief Switches the view panel; with @p_editor, Files shows that tab too.
void showView(App& p_app, FileTab p_view, bool p_editor);
//! @brief Contents of the views (no window of their own).
void scenarioView(App& p_app);
void worldView(App& p_app);
void experimentView(App& p_app);
//! @brief Shows @p_path in the tab of its kind. A scenario of the package is
//! loaded, an experiment becomes the one of the Experiment panel.
void showFile(App& p_app, std::filesystem::path const& p_path);
//! @brief Makes @p_path the experiment of the Experiment panel.
void selectExperiment(App& p_app, std::filesystem::path const& p_path, bool p_focus);
//! @brief Every asset pack found: models, version, license, source.
void assetsPanel(App& p_app);
void filesPanel(App& p_app);

// --- Robot workspace (RobotPanel.cpp, TeachPanel.cpp) -----------------------

void robotPanel(App& p_app);
void plotsPanel(App& p_app);
void teachPanel(App& p_app);
//! @brief Takes the arm from the mission for manual moves.
void takeArm(App& p_app);

// --- Mission workspace (MissionPanels.cpp, SkillsPanel.cpp) -----------------

void timelinePanel(App& p_app);
void skillsPanel(App& p_app);

// --- Autonomy workspace (AutonomyPanels.cpp) --------------------------------

void worldModelPanel(App& p_app);
void perceptionPanel(App& p_app);

// --- Plugin UI (PluginUi.cpp) -----------------------------------------------

//! @brief Loads @p_file at the start of the next frame, never in the middle
//! of one: menus and panels may still be reading the catalog and the old run.
void requestScenario(App& p_app, std::filesystem::path const& p_file);

//! @brief The Demos menu: every package and its scenarios.
void drawDemosMenu(App& p_app);
//! @brief Menus added by the loaded plugin.
void drawPluginMenus(App& p_app);
//! @brief Panels added by the loaded plugin. @p_dock, when not zero, docks
//! them there first; @p_focus brings the first one to the front.
void drawPluginPanels(App& p_app, ImGuiID p_dock, bool p_focus);
