// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#pragma once

#include "SimulatorDisplay.hpp"
#include "SimFrame.hpp"
#include "SimulatorView.hpp"

#include "Robotik/Package/Plugin/PluginSession.hpp"
#include "Robotik/Robot/TeachPendant.hpp"
#include "Robotik/Task/Mission.hpp"
#include "Robotik/Package/Simulation.hpp"
#include "Robotik/Package/Scenario/Experiment.hpp"

#include "Compages/Renderer/Scene/Scene.hpp"
#include "Compages/World/World.hpp"

#include <array>
#include <cstdint>
#include <filesystem>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

// ****************************************************************************
/// A workspace groups the panels of one task. Each one has its own
/// dock layout, saved in imgui.ini.
// ****************************************************************************
enum class Workspace : std::uint8_t
{
    Plugin,   ///< What is loaded: package, scenarios, world, assertions, files.
    Robot,    ///< Joints, limits, plots, teach pendant, cameras.
    Mission,  ///< Behavior tree, skills, timeline, resources.
    Autonomy, ///< Beliefs against truth, perception.
};

inline constexpr std::size_t WORKSPACE_COUNT = 4U;

// ****************************************************************************
/// Recent joint states, for the plots. A ring buffer per series.
// ****************************************************************************
struct JointHistory
{
    static constexpr std::size_t CAPACITY = 3000U;

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
    /// Index of the latest sample, or CAPACITY when empty.
    [[nodiscard]] std::size_t latest() const;
};

// ****************************************************************************
/// A file or a folder of the loaded package (Package panel).
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
/// Tabs of the Files panel: one per kind of file.
// ****************************************************************************
enum class FileTab : std::uint8_t
{
    Scenario,
    World,
    Tree,
    /// The texts of the PDDL planner: domain, problem, output, tree; or a
    /// domain chosen in Files.
    Planner,
    Experiment,
    /// A file of an asset pack clicked in Assets: a model, a picture, a text.
    Asset,
    /// Any other file of the package (plugin.yaml, a URDF...).
    Other,
};

inline constexpr std::size_t FILE_TAB_COUNT = 7U;

// ****************************************************************************
/// A file shown read-only in the Files panel.
// ****************************************************************************
struct ShownFile
{
    std::filesystem::path path;
    std::string text;
};

// ****************************************************************************
/// A model (URDF, STL, glTF) drawn alone in the Asset tab of Files: its own
/// world and scene, and a camera that turns around it with the mouse.
// ****************************************************************************
struct ModelView
{
    std::unique_ptr<compages::world::World> world;
    std::unique_ptr<compages::renderer::Scene> scene;
    compages::world::Entity camera;
    RenderTarget target;
    ViewArea area;
    /// The main loop rendered the model at the size of @c target; until
    /// then the tab shows the background, not an empty texture.
    bool rendered = false;
};

// ****************************************************************************
/// A file of an asset pack shown in the Asset tab of Files: a model in 3D,
/// a picture, or its text.
// ****************************************************************************
struct AssetPreview
{
    std::filesystem::path path;
    std::string text;
    compages::gpu::Texture image;
    /// Set for a model; null for any other file.
    std::unique_ptr<ModelView> model;
    /// A model shows its text instead of the 3D view.
    bool show_text = false;
    /// Why the image could not be read; empty when it was.
    std::string error;
    /// File clicked in Assets, opened at the start of the next frame: the
    /// textures of the file shown may still be drawn in this one.
    std::filesystem::path pending;
    /// The tab was drawn this frame: only then the main loop renders the
    /// model. Cleared before the panels are drawn.
    bool visible = false;
};

// ****************************************************************************
/// A fault injected by hand in the Resources panel.
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
/// A resource seen in fault, sampled once a frame for the Timeline.
// ****************************************************************************
struct FaultSpan
{
    robotik::ResourceId resource = robotik::NO_RESOURCE;
    double start = 0.0;
    double end = 0.0;
    /// Still in fault at the last sample.
    bool open = true;
};

// ****************************************************************************
/// A composite or decorator of the behavior tree that failed, kept with the
/// time it was first seen so its mark stays put on the Timeline.
// ****************************************************************************
struct TreeFailure
{
    std::string label;
    double time = 0.0;
};

/// No phase selected on the Timeline.
inline constexpr std::size_t NO_PHASE = static_cast<std::size_t>(-1);

// ****************************************************************************
/// How the operator looks at the Timeline. Kept across scenario loads.
// ****************************************************************************
struct TimelineView
{
    /// The window of time ends at the present and moves with it.
    bool follow = true;
    /// The window shows the whole run (from 0 to now).
    bool whole_run = false;
    /// Width of the window of time, in simulated seconds.
    double window_s = 30.0;
    /// Left edge of the window when it neither follows nor shows the run.
    double start_s = 0.0;
    /// Hide the skills never requested.
    bool active_only = false;
    /// Show one row per resource.
    bool resources_open = false;
    /// Robots whose rows are folded into their header.
    std::vector<std::string> folded;
};

// ****************************************************************************
/// What the Timeline remembers of the run. Cleared on load and replay.
// ****************************************************************************
struct TimelineMemory
{
    /// Phase of SkillScheduler::trace clicked, or NO_PHASE.
    std::size_t selected = NO_PHASE;
    std::vector<FaultSpan> faults;
    std::vector<TreeFailure> tree_failures;
    /// Tree the failures belong to.
    void const* tree = nullptr;
    /// Simulated time of the last sample; a smaller time is a replay.
    double sampled = -1.0;
};

// ****************************************************************************
/// What the operator looks at: a robot, a skill, a resource or an object of
/// the world model, by name (the
/// names stay valid across a replay). Every panel may set it; the Inspector
/// shows it and the Timeline highlights it. Choosing never commands the
/// robot.
// ****************************************************************************
enum class SelectionKind : std::uint8_t
{
    None,
    Robot,
    Skill,
    Resource,
    Object,
};

struct Selection
{
    SelectionKind kind = SelectionKind::None;
    /// Robot id, skill, resource or object name.
    std::string name;
};

// ****************************************************************************
/// A view that can fill the main area of a workspace (its selector) or the
/// whole work area (Ctrl+Space). Each view is drawn in one place at a time;
/// its own panel then shows a note.
// ****************************************************************************
enum class ViewKind : std::uint8_t
{
    None,
    Scene,    ///< The 3D view.
    Camera,   ///< A robot camera, large.
    Timeline, ///< Skills, resources and faults over time.
    Plots,    ///< Joint positions, velocities, accelerations, efforts.
    Tree,     ///< The behavior tree and the status of its nodes.
    Planner,  ///< The PDDL goal, plan and beliefs.
    Graph,    ///< Plan steps (or tree nodes) and their skills, as a graph.
};

// ****************************************************************************
/// Operator interface state, kept across scenario loads.
// ****************************************************************************
struct UiState
{
    Workspace workspace = Workspace::Mission;
    /// Rebuild the dock layout of the current workspace.
    bool reset_layout = false;
    /// The saved layout of each workspace was checked once (Workspace.cpp).
    std::array<bool, WORKSPACE_COUNT> layout_checked{};
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
    /// Asset tab of Files, and the files of each pack opened in the
    /// Assets panel (pack folder to paths relative to it), read once.
    AssetPreview asset_preview;
    std::map<std::filesystem::path, std::vector<std::filesystem::path>>
        pack_files;
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
    float fault_duration = 0.0F;
    std::vector<ManualFault> manual_faults;
    /// The item chosen in any panel, and the one the Inspector keeps
    /// (kind None: the Inspector follows the selection).
    Selection selection;
    Selection pinned;
    /// Window to bring to the front once the panels are drawn (a click
    /// in the status bar); empty: none.
    std::string focus_window;
    /// Timeline: the view chosen and what it recorded of the run.
    TimelineView timeline;
    TimelineMemory timeline_memory;
    /// The view shown over the work area, and the one under the mouse in
    /// the last frame (what Ctrl+Space would maximize).
    ViewKind maximized = ViewKind::None;
    ViewKind hovered_view = ViewKind::None;
    /// The view maximized in the last frame: focus it only when it opens.
    ViewKind maximized_last = ViewKind::None;
    /// Execution graph: the tree instead of the plan.
    bool graph_tree = false;
    /// Execution graph: the selection it last brought into sight.
    std::string graph_seen;
    /// The view in the main area of each workspace (Plugin has none).
    std::array<ViewKind, WORKSPACE_COUNT> main_view{
        ViewKind::Scene, ViewKind::Scene, ViewKind::Scene, ViewKind::Scene
    };
    /// Scene: a robot camera over its corner, and which one.
    bool inset_camera = true;
    std::size_t inset_index = 0;
    /// Joints drawn in the plots.
    std::vector<bool> plotted;
    float plot_window_s = 10.0F;
    /// Simulated seconds per real second, smoothed.
    float realtime_factor = 0.0F;
};

struct TeachWatch
{
    bool manual = false;
    bool loop = false;
    float joint_step = 0.05F;
    float linear_step = 0.01F;
    float angular_step = 0.05F;
    float duration = 2.0F;
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
    /// The loaded package advances time inside its own update.
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
    float view_x = 0.0F;
    float view_y = 0.0F;
    float view_w = 1.0F;
    float view_h = 1.0F;

    bool playing = false;
    /// --screenshot: the PNG to save of the 3D view, then quit; empty: none.
    std::filesystem::path screenshot;
    /// --screenshot-ui: the PNG to save of the whole window, then quit.
    std::filesystem::path screenshot_ui;
    /// --screenshot: simulated seconds to play before the picture.
    double screenshot_at = 3.0;
    bool step_once = false;
    float speed = 1.0F;
    std::uint64_t seed = 0;
    /// When set, replaces @c robot.tool read from the scenario file.
    bool tool_override = false;
    std::string tool_override_name;

    /// Stops the plugin before the simulation it listens to is freed.
    ~App();

    void load(bool p_keep_tool = false);
    void reset(std::uint64_t p_seed);
    void advance(double p_elapsed);

private:

    void step(double p_elapsed);

    double m_lag = 0.0;
    bool m_reported = false;
};

void drawPanels(App& p_app);
