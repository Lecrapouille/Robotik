// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#pragma once

#include "SimulatorView.hpp"

#include "Robotik/Robot/TeachPendant.hpp"
#include "Robotik/Plugin/PluginSession.hpp"
#include "Robotik/Runtime/Mission.hpp"
#include "Robotik/Runtime/Simulation.hpp"
#include "Robotik/Scenario/Experiment.hpp"

#include "Compages/Renderer/Scene.hpp"
#include "Compages/World/World.hpp"

#include <array>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <vector>

// ****************************************************************************
/// @brief A workspace groups the panels of one task. Each one has its own
/// dock layout, saved in imgui.ini.
// ****************************************************************************
enum class Workspace : std::uint8_t
{
    Plugin,   ///< What is loaded: package, scenarios, world, assertions, files.
    Robot,    ///< Joints, limits, plots, teach pendant, cameras.
    Mission,  ///< Behavior tree, skills, timeline, resources.
    Autonomy, ///< Beliefs against truth, perception.
};

inline constexpr std::size_t WORKSPACE_COUNT = 4u;

// ****************************************************************************
/// @brief Recent joint states, for the plots. A ring buffer per series.
// ****************************************************************************
struct JointHistory
{
    static constexpr std::size_t CAPACITY = 3000u;

    std::vector<float> time;
    std::vector<std::vector<float>> position;
    std::vector<std::vector<float>> velocity;
    std::vector<std::vector<float>> acceleration;
    std::vector<std::vector<float>> effort;
    /// Index of the oldest sample once the buffer is full.
    std::size_t head = 0;
    std::size_t count = 0;

    void clear();
    void record(robotik::Simulation const& p_simulation);
    /// @brief Index of the latest sample, or CAPACITY when empty.
    [[nodiscard]] std::size_t latest() const;
};

// ****************************************************************************
/// @brief A file or a folder of the loaded package (Package panel).
// ****************************************************************************
struct PackageEntry
{
    std::string name;
    std::filesystem::path path;
    std::uintmax_t size = 0;
    bool folder = false;
    std::vector<PackageEntry> children;
};

// ****************************************************************************
/// @brief Tabs of the Files panel: one per kind of file.
// ****************************************************************************
enum class FileTab : std::uint8_t
{
    Scenario,
    World,
    Tree,
    /// The texts of the PDDL planner: domain, problem, output, tree.
    Planner,
    Experiment,
    /// Any other file of the package (plugin.yaml, a URDF...).
    Other,
};

inline constexpr std::size_t FILE_TAB_COUNT = 6u;

// ****************************************************************************
/// @brief A file shown read-only in the Files panel.
// ****************************************************************************
struct ShownFile
{
    std::filesystem::path path;
    std::string text;
};

// ****************************************************************************
/// @brief A fault injected by hand in the Resources panel.
// ****************************************************************************
struct ManualFault
{
    std::string resource;
    /// Simulation time of the injection.
    double since = 0.0;
    /// Simulation time of the automatic restore; 0: none.
    double until = 0.0;
};

// ****************************************************************************
/// @brief What a resource chip of the Timeline shows. Skills take and give
/// back resources within a few steps: the last holders stay shown a short
/// time so the chip does not flicker.
// ****************************************************************************
struct ChipHold
{
    std::size_t holders = 0;
    bool exclusive = false;
    /// Wall-clock time (ImGui::GetTime) until which it is kept.
    double until = 0.0;
};

// ****************************************************************************
/// @brief Operator interface state, kept across scenario loads.
// ****************************************************************************
struct UiState
{
    Workspace workspace = Workspace::Mission;
    /// Rebuild the dock layout of the current workspace.
    bool reset_layout = false;
    /// Dock the plugin panels in the current workspace, then focus them.
    bool place_plugin_panels = false;
    bool focus_plugin_panel = false;
    bool show_shortcuts = false;
    bool show_open = false;
    /// Scenario to load at the start of the next frame.
    std::filesystem::path pending_scenario;
    std::string open_path;
    /// Package panel: the files of the package, read once per package.
    std::filesystem::path package_root;
    PackageEntry package_tree;
    /// Files panel: a file shown in a tab instead of the loaded one (empty
    /// path: the loaded one). Scenario, World and Tree are reset when
    /// another scenario is loaded.
    std::array<ShownFile, FILE_TAB_COUNT> shown_files;
    std::filesystem::path shown_for;
    /// Tab to bring to the front on the next frame.
    std::optional<FileTab> select_tab;
    /// Experiment panel: the experiment chosen in Files, read once.
    std::filesystem::path experiment_root;
    std::filesystem::path experiment_path;
    std::optional<robotik::Experiment> experiment;
    std::string experiment_error;
    /// The panel beside Files shows one view: Scenario, World, Tree,
    /// Planner or Experiment. It follows the tab chosen in Files.
    FileTab view = FileTab::Scenario;
    std::optional<FileTab> active_tab;
    /// Assets panel: text filter, and only the packs of the running scenario.
    std::array<char, 64> asset_filter{};
    bool assets_in_use = false;
    /// Planner: plan whose texts are shown (-1: the last one), result of
    /// Save PDDL, and the beliefs filter.
    int planner_record = -1;
    std::string planner_saved;
    bool beliefs_only_true = false;
    /// Skills panel: cards (false) or a table (true).
    bool skills_table = false;
    /// Resources panel: resources ticked for the next fault, its duration
    /// and the faults injected by hand.
    std::vector<bool> fault_pick;
    float fault_duration = 0.0f;
    std::vector<ManualFault> manual_faults;
    /// Timeline: what each resource chip shows.
    std::vector<ChipHold> chip_hold;
    /// Joints drawn in the plots.
    std::vector<bool> plotted;
    float plot_window_s = 10.0f;
    /// Simulated seconds per real second, smoothed.
    float realtime_factor = 0.0f;
};

struct TeachWatch
{
    bool manual = false;
    bool loop = false;
    float joint_step = 0.05f;
    float linear_step = 0.01f;
    float angular_step = 0.05f;
    float duration = 2.0f;
    std::string label;
    robotik::TeachPendant pendant;
    std::vector<compages::world::Entity> markers;
    std::uint32_t marker_serial = 0;
    /// X, Y, Z arrows on the frame the pendant moves (Robot workspace).
    bool show_axes = true;
    bool axes_made = false;
    std::array<compages::world::Entity, 3> axes{};
};

struct App
{
    std::filesystem::path scenario_path;
    std::string scenario_title;
    std::string scenario_text;
    std::filesystem::path world_path;
    std::string world_text;
    std::string tree_text;
    std::string error;
    /// @brief The loaded package advances time inside its own update.
    bool owns_clock = false;

    robotik::PluginSession plugins;

    std::unique_ptr<compages::world::World> world;
    std::unique_ptr<compages::renderer::Scene> scene;
    std::unique_ptr<SimulatorView> scene_view;
    std::unique_ptr<robotik::Mission> mission;
    std::unique_ptr<robotik::Simulation> simulation;
    TeachWatch teach;
    UiState ui;
    JointHistory history;
    compages::world::Entity view_camera;

    RenderTarget view;
    bool view_hovered = false;
    /// Top-left corner and size of the 3D view on the screen (ImGui pixels):
    /// the camera rig turns the mouse into a point of the ground.
    float view_x = 0.0f;
    float view_y = 0.0f;
    float view_w = 1.0f;
    float view_h = 1.0f;

    bool playing = false;
    /// --screenshot: the PNG to save of the 3D view, then quit; empty: none.
    std::filesystem::path screenshot;
    /// --screenshot: simulated seconds to play before the picture.
    double screenshot_at = 3.0;
    bool step_once = false;
    float speed = 1.0f;
    std::uint64_t seed = 0;
    /// @brief When set, replaces @c robot.tool read from the scenario file.
    bool tool_override = false;
    std::string tool_override_name;

    void load(bool p_keep_tool = false);
    void reset(std::uint64_t p_seed);
    void advance(double p_elapsed);

private:

    void step(double p_elapsed);

    double m_lag = 0.0;
    bool m_reported = false;
};

void drawPanels(App& p_app);
