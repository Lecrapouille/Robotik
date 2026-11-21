// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.
#include "Ui.hpp"

#include "imgui_stdlib.h"
#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_internal.h>

#include <algorithm>
#include <array>
#include <cstdio>
#include <filesystem>
#include <initializer_list>
#include <string>
#include <string_view>
#include <vector>

/// Bump it when a default layout or default editors change: the layouts and
/// the open editors saved in simulator.ini are then rebuilt once.
static constexpr unsigned LAYOUT_VERSION = 23U;

/// Title, shortcut and hint of one workspace.
struct WorkspaceInfo
{
    char const* name;
    char const* shortcut;
    /// A panel of the column where the plugin panels go.
    char const* anchor;
    char const* hint;
};

static std::array<WorkspaceInfo, WORKSPACE_COUNT> const WORKSPACES{ {
    { "Plugin",
      "Ctrl+1",
      "Files",
      "Read the loaded plugin: its files, dependencies and licenses, the "
      "scenario, "
      "its behavior tree, the experiments, the world and the asset packs." },
    { "Robot",
      "Ctrl+2",
      "Joints",
      "The robot body: joints and their limits, plots, manual moves, "
      "cameras." },
    { "Mission",
      "Ctrl+3",
      "Resources",
      "The execution graph, skills, resources and their timeline, the "
      "PDDL planner." },
    { "Autonomy",
      "Ctrl+4",
      "World model",
      "What the robot believes against the ground truth, and what it "
      "perceives." },
} };

/// Title, shortcut and hint of one workspace.
static WorkspaceInfo const& infoOf(Workspace p_workspace)
{
    return WORKSPACES[static_cast<std::size_t>(p_workspace)];
}

void selectWorkspace(App& p_app, Workspace p_workspace)
{
    // Applied by updatePanelLayout, between two frames.
    p_app.ui.pending_workspace = p_workspace;
    p_app.ui.pending_maximized = std::string();
}

// --- Optional windows --------------------------------------------------------

/// One editor, with a stable name and its drawing function.
struct PanelInfo
{
    char const* name;
    void (*draw)(App&);
};

static std::array<PanelInfo, PANEL_COUNT> const PANELS{ {
    { "3D scene", scenePanel },
    { "Execution graph", graphPanel },
    { "Plugin", pluginPanel },
    { "Files", filesPanel },
    { "Package details", viewPanel },
    { "Assets", assetsPanel },
    { "Inspector", [](App& app) { inspectorPanel(app, ""); } },
    { "Joints", robotPanel },
    { "Joint plots", plotsPanel },
    { "Teach Pendant", teachPanel },
    { "Cameras", [](App& app) { camerasPanel(app, ""); } },
    { "Assertions", [](App& app) { assertionsPanel(app, ""); } },
    { "Timeline", timelinePanel },
    { "Events", eventsPanel },
    { "Resources", resourcesPanel },
    { "Planner", plannerPanel },
    { "World model", worldModelPanel },
    { "Perception", perceptionPanel },
} };

char const* panelName(PanelKind p_panel)
{
    return PANELS[static_cast<std::size_t>(p_panel)].name;
}

PanelVisibility defaultPanels(Workspace p_workspace)
{
    PanelVisibility result{};
    auto open = [&](std::initializer_list<PanelKind> p_panels)
    {
        for (PanelKind const panel : p_panels)
            result[static_cast<std::size_t>(panel)] = true;
    };
    // No Inspector: the first click on a name opens it (itemLink).
    switch (p_workspace)
    {
        case Workspace::Plugin:
            open({ PanelKind::Plugin,
                   PanelKind::Files,
                   PanelKind::PackageView,
                   PanelKind::Assets });
            break;
        case Workspace::Robot:
            open({ PanelKind::Scene,
                   PanelKind::Joints,
                   PanelKind::Teach,
                   PanelKind::Plots });
            break;
        case Workspace::Mission:
            // The Planner is added when the scenario plans (drawPanels).
            open({ PanelKind::Graph,
                   PanelKind::Scene,
                   PanelKind::Resources,
                   PanelKind::Assertions,
                   PanelKind::Timeline,
                   PanelKind::Events });
            break;
        case Workspace::Autonomy:
            open({ PanelKind::Scene,
                   PanelKind::Cameras,
                   PanelKind::WorldModel,
                   PanelKind::Perception });
            break;
    }
    return result;
}

/// Rebuild positions; only an explicit restore reopens default editors.
static void arrangeWorkspace(App& p_app, bool p_defaults)
{
    p_app.ui.reset_layout = true;
    p_app.ui.pending_maximized = std::string();
    p_app.ui.place_plugin_panels = true;
    if (p_defaults)
        p_app.ui.panels = defaultPanels(p_app.ui.workspace);
    p_app.ui.workspace_layouts[static_cast<std::size_t>(p_app.ui.workspace)]
        .clear();
    ImGui::MarkIniSettingsDirty();
}

/// Opening an editor always focuses its existing window.
void showPanel(App& p_app, PanelKind p_panel)
{
    std::size_t const index = static_cast<std::size_t>(p_panel);
    p_app.ui.panels[index] = true;
    if (!p_app.ui.maximized_window.empty() &&
        p_app.ui.maximized_window != PANELS[index].name)
        p_app.ui.pending_maximized = std::string();
    p_app.ui.focus_windows.push_back(PANELS[index].name);
    ImGui::MarkIniSettingsDirty();
}

/// Unified editor list: opening an existing editor never creates a copy.
static void panelMenu(App& p_app)
{
    for (std::size_t i = 0; i < PANEL_COUNT; ++i)
        if (ImGui::MenuItem(PANELS[i].name, nullptr, p_app.ui.panels[i]))
            showPanel(p_app, static_cast<PanelKind>(i));
    if (!p_app.plugins.host().panels().empty())
        ImGui::SeparatorText("Plugin windows");
    for (robotik::PluginPanel const& panel : p_app.plugins.host().panels())
    {
        bool& open = p_app.ui.plugin_panels
                         .try_emplace(pluginEditorKey(p_app, panel), true)
                         .first->second;
        if (ImGui::MenuItem(panel.title.c_str(), nullptr, open))
        {
            open = true;
            p_app.ui.pending_maximized = std::string();
            p_app.ui.focus_windows.push_back(pluginWindowTitle(p_app, panel));
            ImGui::MarkIniSettingsDirty();
        }
    }
}

void toggleMaximized(App& p_app, std::string const& p_window)
{
    p_app.ui.pending_maximized =
        p_app.ui.maximized_window == p_window ? std::string() : p_window;
}

/// True when the requested editor still exists and is open.
static bool editorOpen(App const& p_app, std::string const& p_title)
{
    for (std::size_t i = 0; i < PANEL_COUNT; ++i)
        if (p_title == PANELS[i].name)
            return p_app.ui.panels[i];
    for (robotik::PluginPanel const& panel : p_app.plugins.host().panels())
        if (p_title == pluginWindowTitle(p_app, panel))
        {
            auto const found =
                p_app.ui.plugin_panels.find(pluginEditorKey(p_app, panel));
            return found == p_app.ui.plugin_panels.end() || found->second;
        }
    return false;
}

static void applyMaximized(App& p_app);

/// Applies a dock layout kept as ini text: positions, docks and tab order.
/// What the text also holds of the editors (open ones, workspace, plugin
/// panels, the editors of each workspace) stays as it is now.
static void loadLayout(UiState& p_ui, std::string const& p_layout)
{
    Workspace const workspace = p_ui.workspace;
    auto const panels = p_ui.panels;
    auto const plugins = p_ui.plugin_panels;
    auto const remembered = p_ui.workspace_panels;
    ImGui::LoadIniSettingsFromMemory(p_layout.c_str(), p_layout.size());
    p_ui.workspace = workspace;
    p_ui.panels = panels;
    p_ui.plugin_panels = plugins;
    p_ui.workspace_panels = remembered;
}

/// Leaves the current workspace for @p_workspace: the one left keeps its
/// editors and layout, the other gets back what it had, or its defaults
/// the first time. Choosing the current one again restores its defaults.
static void switchWorkspace(App& p_app, Workspace p_workspace)
{
    UiState& ui = p_app.ui;
    std::size_t const from = static_cast<std::size_t>(ui.workspace);
    std::size_t const to = static_cast<std::size_t>(p_workspace);
    if (from == to)
    {
        ui.panels = defaultPanels(p_workspace);
        ui.workspace_layouts[to].clear();
        ui.reset_layout = true;
    }
    else
    {
        ui.workspace_panels[from] = ui.panels;
        ui.workspace_layouts[from] = ImGui::SaveIniSettingsToMemory();
        ui.workspace = p_workspace;
        ui.panels =
            ui.workspace_panels[to].value_or(defaultPanels(p_workspace));
        if (ui.workspace_layouts[to].empty())
            ui.reset_layout = true;
        else
            loadLayout(ui, ui.workspace_layouts[to]);
    }
    ui.focused_window.clear();
    ui.place_plugin_panels = true;
    if (p_workspace == Workspace::Robot && p_app.simulation &&
        ui.selection.kind == SelectionKind::None &&
        !p_app.simulation->cell().robots().empty())
        ui.selection = { SelectionKind::Robot,
                         p_app.simulation->cell().robots().front()->id() };
    ImGui::MarkIniSettingsDirty();
}

void updatePanelLayout(App& p_app)
{
    UiState& ui = p_app.ui;
    if (ImGui::GetFrameCount() == 0)
        return;
    // A new workspace first brings an expanded editor back to the layout.
    if (ui.pending_workspace && !ui.maximized_window.empty())
        ui.pending_maximized = std::string();
    if (ui.pending_maximized)
        applyMaximized(p_app);
    if (ui.pending_workspace)
    {
        Workspace const workspace = *ui.pending_workspace;
        ui.pending_workspace.reset();
        switchWorkspace(p_app, workspace);
    }
}

/// Expands the editor asked for, or brings the layout back.
static void applyMaximized(App& p_app)
{
    UiState& ui = p_app.ui;
    std::string const requested = *ui.pending_maximized;
    ui.pending_maximized.reset();
    if (!ui.maximized_window.empty())
    {
        // Reapply the complete dock tree, including empty splits and tab order.
        // Preserve any editor opened or closed while the window was expanded.
        loadLayout(ui, ui.saved_layout);
        ImGui::GetIO().IniFilename = ui.settings_filename;
        ui.saved_layout.clear();
        ui.maximized_window.clear();
        ImGui::MarkIniSettingsDirty();
    }
    if (!requested.empty() && editorOpen(p_app, requested))
    {
        ui.saved_layout = ImGui::SaveIniSettingsToMemory();
        ui.settings_filename = ImGui::GetIO().IniFilename;
        // Never persist the temporary undocking, even on an automatic save.
        ImGui::GetIO().IniFilename = nullptr;
        ui.maximized_window = requested;
    }
}

/// Expand or restore icon in the top-right corner of the current editor.
/// It is tested before the content, so it wins the mouse over a 3D view,
/// and drawn after it by drawCorners, so it stays on top.
static void cornerButton(App& p_app, char const* p_title, bool p_expanded)
{
    ImGuiWindow* window = ImGui::GetCurrentWindow();
    float const size = ImGui::GetFrameHeight();
    ImVec2 const min(window->InnerRect.Max.x - size - 6.0F,
                     window->InnerRect.Min.y + 6.0F);
    ImRect const box(min, ImVec2(min.x + size, min.y + size));
    ImGuiID const id = window->GetID("##expand");
    if (!ImGui::ItemAdd(box, id))
        return;
    bool hovered = false;
    bool held = false;
    if (ImGui::ButtonBehavior(box, id, &hovered, &held))
        toggleMaximized(p_app, p_title);
    ImGui::SetItemTooltip(p_expanded ? "Back to the layout  (Esc)"
                                     : "Over the whole work area  "
                                       "(Ctrl+Space)");
    p_app.ui.corners.push_back(
        { p_title, min.x, min.y, size, p_expanded, hovered || held });
}

/// Draws the corner buttons over the content of their editors.
static void drawCorners(App& p_app)
{
    for (UiState::Corner const& corner : p_app.ui.corners)
    {
        ImGuiWindow const* window =
            ImGui::FindWindowByName(corner.window.c_str());
        if (window == nullptr)
            continue;
        ImDrawList* draw = window->DrawList;
        draw->PushClipRect(window->InnerRect.Min, window->InnerRect.Max, true);
        drawIconBox(draw,
                    ImVec2(corner.x, corner.y),
                    corner.size,
                    corner.expanded ? Icon::Restore : Icon::Expand,
                    corner.hovered);
        draw->PopClipRect();
    }
    p_app.ui.corners.clear();
}

bool beginEditor(App& p_app,
                 char const* p_title,
                 bool& p_open,
                 ImGuiWindowFlags p_flags)
{
    UiState& ui = p_app.ui;
    bool const expanded = ui.maximized_window == p_title;
    if (expanded)
    {
        ImGuiViewport const* viewport = ImGui::GetMainViewport();
        ImGui::SetNextWindowDockID(0);
        ImGui::SetNextWindowPos(viewport->WorkPos);
        ImGui::SetNextWindowSize(viewport->WorkSize);
        p_flags |= ImGuiWindowFlags_NoDocking | ImGuiWindowFlags_NoMove |
                   ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse |
                   ImGuiWindowFlags_NoBringToFrontOnFocus |
                   ImGuiWindowFlags_NoSavedSettings;
    }
    bool const visible = ImGui::Begin(p_title, &p_open, p_flags);
    if (ImGui::IsWindowFocused(ImGuiFocusedFlags_RootAndChildWindows))
        ui.focused_window = p_title;
    if (!p_open)
    {
        if (ui.focused_window == p_title)
            ui.focused_window.clear();
        if (expanded)
            ui.pending_maximized = std::string();
        ImGui::MarkIniSettingsDirty();
    }
    if (ImGui::BeginPopupContextItem("editor_actions"))
    {
        if (ImGui::MenuItem(expanded ? "Restore" : "Expand", "Ctrl+Space"))
            toggleMaximized(p_app, p_title);
        if (ImGui::MenuItem("Close"))
        {
            p_open = false;
            if (ui.focused_window == p_title)
                ui.focused_window.clear();
            if (expanded)
                ui.pending_maximized = std::string();
            ImGui::MarkIniSettingsDirty();
        }
        ImGui::EndPopup();
    }
    if (visible)
        cornerButton(p_app, p_title, expanded);
    return visible;
}

bool beginPanel(App& p_app,
                PanelKind p_panel,
                char const* p_title,
                ImGuiWindowFlags p_flags)
{
    return beginEditor(p_app,
                       p_title,
                       p_app.ui.panels[static_cast<std::size_t>(p_panel)],
                       p_flags);
}

void initializePanelSettings(App& p_app)
{
    ImGuiSettingsHandler handler;
    handler.TypeName = "RobotikEditors1";
    handler.TypeHash = ImHashStr(handler.TypeName);
    handler.UserData = &p_app.ui;
    handler.ReadOpenFn =
        [](ImGuiContext*, ImGuiSettingsHandler* p_handler, char const*) -> void*
    {
        static_cast<UiState*>(p_handler->UserData)->settings_version = 0U;
        return p_handler->UserData;
    };
    handler.ReadLineFn = [](ImGuiContext*,
                            ImGuiSettingsHandler*,
                            void* p_entry,
                            char const* p_line)
    {
        auto& ui = *static_cast<UiState*>(p_entry);
        unsigned mask = 0, preset = 0;
        // Written first. Editors saved by another version of the layouts
        // are not read: that version's defaults would be shown.
        if (std::sscanf(p_line, "Version=%u", &preset) == 1)
            ui.settings_version = preset;
        if (ui.settings_version != LAYOUT_VERSION)
            return;
        if (std::sscanf(p_line, "Open=%u", &mask) == 1)
            for (std::size_t i = 0; i < PANEL_COUNT; ++i)
                ui.panels[i] = (mask & (1U << i)) != 0;
        if (std::sscanf(p_line, "Preset=%u", &preset) == 1 &&
            preset < WORKSPACE_COUNT)
            ui.workspace = static_cast<Workspace>(preset);
        // The editors each workspace had when it was left.
        if (std::sscanf(p_line, "Workspace%u=%u", &preset, &mask) == 2 &&
            preset < WORKSPACE_COUNT)
        {
            PanelVisibility panels{};
            for (std::size_t i = 0; i < PANEL_COUNT; ++i)
                panels[i] = (mask & (1U << i)) != 0;
            ui.workspace_panels[preset] = panels;
        }
        if (std::string_view(p_line).starts_with("ClosedPlugin="))
            ui.plugin_panels[p_line + 13] = false;
    };
    handler.WriteAllFn = [](ImGuiContext*,
                            ImGuiSettingsHandler* p_handler,
                            ImGuiTextBuffer* p_buffer)
    {
        auto const& ui = *static_cast<UiState*>(p_handler->UserData);
        auto maskOf = [](PanelVisibility const& p_panels)
        {
            unsigned mask = 0;
            for (std::size_t i = 0; i < PANEL_COUNT; ++i)
                if (p_panels[i])
                    mask |= 1U << i;
            return mask;
        };
        p_buffer->appendf("[%s][Visibility]\nVersion=%u\nOpen=%u\nPreset=%u\n",
                          p_handler->TypeName,
                          LAYOUT_VERSION,
                          maskOf(ui.panels),
                          static_cast<unsigned>(ui.workspace));
        for (std::size_t i = 0; i < WORKSPACE_COUNT; ++i)
            if (ui.workspace_panels[i])
                p_buffer->appendf(
                    "Workspace%zu=%u\n", i, maskOf(*ui.workspace_panels[i]));
        for (auto const& [name, open] : ui.plugin_panels)
            if (!open)
                p_buffer->appendf("ClosedPlugin=%s\n", name.c_str());
        p_buffer->append("\n");
    };
    ImGui::AddSettingsHandler(&handler);
}

void shutdownPanelSettings(App& p_app)
{
    p_app.ui.pending_maximized = std::string();
    updatePanelLayout(p_app);
    if (ImGui::GetIO().IniFilename != nullptr)
        ImGui::SaveIniSettingsToDisk(ImGui::GetIO().IniFilename);
    ImGui::RemoveSettingsHandler("RobotikEditors1");
    ImGui::GetIO().IniFilename = nullptr;
}

// --- Layouts ----------------------------------------------------------------

/// Splits @p_node; the first returned id is the new side.
static ImGuiID split(ImGuiID& p_node, ImGuiDir p_dir, float p_ratio)
{
    return ImGui::DockBuilderSplitNode(
        p_node, p_dir, p_ratio, nullptr, &p_node);
}

/// Docks a window on a dock-builder node.
static void dock(std::string const& p_window, ImGuiID p_node)
{
    ImGui::DockBuilderDockWindow(p_window.c_str(), p_node);
}

/// Asks for @p_window to be the visible tab of its group once the layout
/// is built (ImGui would show the last one docked); see frontTabs.
static void front(std::vector<std::string>& p_front, char const* p_window)
{
    p_front.emplace_back(p_window);
}

/// Docks every window not placed yet by a workspace: the 3D scene in
/// @p_scene, the wide ones (Timeline, Events, Joint plots) in @p_wide, the
/// package windows in @p_package, the others in @p_side; @p_placed are
/// docked by the caller. Closed windows take no space.
static void dockRest(ImGuiID p_scene,
                     ImGuiID p_side,
                     ImGuiID p_wide,
                     ImGuiID p_package,
                     std::initializer_list<PanelKind> p_placed = {})
{
    for (std::size_t i = 0; i < PANEL_COUNT; ++i)
    {
        PanelKind const kind = static_cast<PanelKind>(i);
        char const* name = PANELS[i].name;
        if (std::ranges::find(p_placed, kind) != p_placed.end())
            continue;
        if (kind == PanelKind::Scene || kind == PanelKind::Graph)
            dock(name, p_scene);
        else if (kind == PanelKind::Timeline || kind == PanelKind::Events ||
                 kind == PanelKind::Plots)
            dock(name, p_wide);
        else if (kind == PanelKind::Files || kind == PanelKind::Plugin ||
                 kind == PanelKind::PackageView || kind == PanelKind::Assets)
            dock(name, p_package);
        else
            dock(name, p_side);
    }
}

/// Builds the dock layout of one workspace. @p_graph: the scenario has a
/// behavior tree or a plan to draw. @p_front receives the tabs to show.
static void layout(Workspace p_workspace,
                   ImGuiID p_dock,
                   bool p_graph,
                   std::vector<std::string>& p_front)
{
    ImGui::DockBuilderRemoveNode(p_dock);
    ImGui::DockBuilderAddNode(p_dock, ImGuiDockNodeFlags_DockSpace);
    ImGui::DockBuilderSetNodeSize(p_dock, ImGui::GetMainViewport()->WorkSize);
    ImGuiID center = p_dock;
    switch (p_workspace)
    {
        case Workspace::Plugin:
        {
            // No 3D view: this workspace reads the package. The plugin on
            // the left, the files in the middle, beside them one panel for
            // the view of the file read (scenario, world, tree, experiment).
            ImGuiID const left = split(center, ImGuiDir_Left, 0.24F);
            ImGuiID right = split(center, ImGuiDir_Right, 0.40F);
            ImGuiID const assets = split(right, ImGuiDir_Down, 0.30F);
            dock("Plugin", left);
            dock("Files", center);
            dock("Package details", right);
            dock("Assets", assets);
            front(p_front, "Package details");
            dockRest(center,
                     right,
                     right,
                     center,
                     { PanelKind::Plugin,
                       PanelKind::Files,
                       PanelKind::PackageView,
                       PanelKind::Assets });
            break;
        }
        case Workspace::Robot:
        {
            // The body: a small scene above its joint plots on the left,
            // the joint controls and the Teach Pendant on the right.
            ImGuiID const right = split(center, ImGuiDir_Right, 0.48F);
            ImGuiID const bottom = split(center, ImGuiDir_Down, 0.50F);
            dockRest(center, right, bottom, center);
            front(p_front, "Joints");
            break;
        }
        case Workspace::Mission:
        {
            // Execution and planning share the middle; diagnostics stay
            // beside the small scene, with the Timeline below.
            ImGuiID right = split(center, ImGuiDir_Right, 0.30F);
            ImGuiID const bottom = split(center, ImGuiDir_Down, 0.30F);
            ImGuiID const side =
                p_graph ? split(right, ImGuiDir_Down, 0.60F) : right;
            dock(p_graph ? "Execution graph" : "3D scene", center);
            dock("Planner", center);
            if (p_graph)
                dock("3D scene", right);
            dockRest(
                center,
                side,
                bottom,
                center,
                { PanelKind::Scene, PanelKind::Graph, PanelKind::Planner });
            // An empty graph comes last among the tabs of the side.
            if (!p_graph)
                dock("Execution graph", side);
            front(p_front, p_graph ? "Resources" : "Execution graph");
            front(p_front, "Timeline");
            break;
        }
        case Workspace::Autonomy:
        {
            // Perception: the scene and the cameras the same size above,
            // the beliefs against the truth and the detections below.
            ImGuiID bottom = split(center, ImGuiDir_Down, 0.45F);
            ImGuiID const right = split(center, ImGuiDir_Right, 0.50F);
            ImGuiID const detections = split(bottom, ImGuiDir_Right, 0.45F);
            dock("Cameras", right);
            dock("World model", bottom);
            dock("Perception", detections);
            dockRest(center,
                     right,
                     bottom,
                     center,
                     { PanelKind::Cameras,
                       PanelKind::WorldModel,
                       PanelKind::Perception });
            front(p_front, "Cameras");
            front(p_front, "World model");
            break;
        }
    }
    ImGui::DockBuilderFinish(p_dock);
}

// --- Commands ---------------------------------------------------------------

/// Loads the current scenario again.
static void reload(App& p_app)
{
    if (!p_app.scenario_path.empty())
    {
        p_app.load(true);
    }
}

/// Asks the window to close.
static void quit()
{
    if (GLFWwindow* window = glfwGetCurrentContext())
    {
        glfwSetWindowShouldClose(window, GLFW_TRUE);
    }
}

/// Keyboard shortcuts that are not ImGui menu chords.
static void shortcuts(App& p_app)
{
    ImGuiIO const& io = ImGui::GetIO();
    if (io.WantTextInput)
    {
        return;
    }
    bool const ctrl = io.KeyCtrl;
    for (std::size_t i = 0; i < WORKSPACE_COUNT; ++i)
    {
        if (ctrl &&
            ImGui::IsKeyPressed(
                static_cast<ImGuiKey>(ImGuiKey_1 + static_cast<int>(i)), false))
        {
            selectWorkspace(p_app, static_cast<Workspace>(i));
        }
    }
    if (ctrl && ImGui::IsKeyPressed(ImGuiKey_O, false))
    {
        p_app.ui.show_open = true;
    }
    if (ctrl && ImGui::IsKeyPressed(ImGuiKey_Q, false))
    {
        quit();
    }
    if (ImGui::IsKeyPressed(ImGuiKey_F5, false))
    {
        reload(p_app);
    }
    if (ImGui::IsKeyPressed(ImGuiKey_F1, false))
    {
        p_app.ui.show_shortcuts = true;
    }
    if (ctrl && ImGui::IsKeyPressed(ImGuiKey_Space, false))
    {
        if (!p_app.ui.maximized_window.empty())
            p_app.ui.pending_maximized = std::string();
        else if (!p_app.ui.focused_window.empty())
            toggleMaximized(p_app, p_app.ui.focused_window);
    }
    if (!p_app.ui.maximized_window.empty() &&
        ImGui::IsKeyPressed(ImGuiKey_Escape, false))
        p_app.ui.pending_maximized = std::string();
    if (!p_app.simulation)
    {
        return;
    }
    if (!ctrl && ImGui::IsKeyPressed(ImGuiKey_Space, false))
    {
        p_app.playing = !p_app.playing;
    }
    if (!ctrl && !p_app.playing && ImGui::IsKeyPressed(ImGuiKey_Period, true))
    {
        p_app.step_once = true;
    }
    if (ctrl && ImGui::IsKeyPressed(ImGuiKey_R, false))
    {
        p_app.reset(p_app.seed);
    }
    if (ctrl && ImGui::IsKeyPressed(ImGuiKey_N, false))
    {
        p_app.reset(p_app.seed + 1U);
    }
    if (ctrl && ImGui::IsKeyPressed(ImGuiKey_E, false))
    {
        toggleEmergency(p_app);
    }
}

// --- Menu bar ---------------------------------------------------------------

/// Why a workspace tab is drawn in red, one reason per line; empty
/// when all is well. Plugin: the scenario did not load, or a package of the
/// catalog is broken. Mission: the run failed, the robot is stopped, or a
/// resource is in fault.
static std::string alertOf(App const& p_app, Workspace p_workspace)
{
    std::string alert;
    auto line = [&alert](std::string const& p_text)
    {
        alert += alert.empty() ? "" : "\n";
        alert += p_text;
    };
    if (p_workspace == Workspace::Plugin)
    {
        if (!p_app.error.empty())
        {
            line(p_app.error);
        }
        for (std::string const& error : p_app.plugins.catalog().errors())
        {
            line(error);
        }
    }
    else if (p_workspace == Workspace::Mission && p_app.simulation)
    {
        robotik::Simulation const& simulation = *p_app.simulation;
        switch (simulation.outcome())
        {
            case robotik::Simulation::Outcome::ControllerFailed:
                line("The controller failed.");
                break;
            case robotik::Simulation::Outcome::Timeout:
                line("The run timed out.");
                break;
            case robotik::Simulation::Outcome::AssertionFailed:
                line("An assertion failed.");
                break;
            case robotik::Simulation::Outcome::ControllerSucceeded:
            case robotik::Simulation::Outcome::StopCondition:
                if (!simulation.passed())
                {
                    line("The run ended without passing its assertions.");
                }
                break;
            case robotik::Simulation::Outcome::Running:
            case robotik::Simulation::Outcome::Interrupted:
                break;
        }
        if (emergencyEngaged(p_app))
        {
            line("Emergency stop engaged.");
        }
        robotik::ResourceManager const& resources =
            simulation.cell().robot().resources();
        for (std::size_t i = 0; i < resources.size(); ++i)
        {
            robotik::ResourceId const id = static_cast<robotik::ResourceId>(i);
            if (!resources.available(id))
            {
                line("Resource in fault: " + resources.name(id));
            }
        }
    }
    return alert;
}

/// The four workspace tabs, centred in the menu bar.
static void workspaceTabs(App& p_app)
{
    // Centred in the free part of the menu bar.
    float width = 0.0F;
    for (WorkspaceInfo const& info : WORKSPACES)
    {
        width += ImGui::CalcTextSize(info.name).x +
                 ImGui::GetStyle().FramePadding.x * 4.0F;
    }
    float const center = (ImGui::GetWindowWidth() - width) * 0.5F;
    if (center > ImGui::GetCursorPosX())
    {
        ImGui::SetCursorPosX(center);
    }
    for (std::size_t i = 0; i < WORKSPACE_COUNT; ++i)
    {
        WorkspaceInfo const& info = WORKSPACES[i];
        bool const current = p_app.ui.workspace == static_cast<Workspace>(i);
        std::string const alert = alertOf(p_app, static_cast<Workspace>(i));
        ImGui::PushStyleColor(ImGuiCol_Button,
                              current
                                  ? ImGui::GetStyleColorVec4(ImGuiCol_FrameBg)
                                  : ImVec4(0.0F, 0.0F, 0.0F, 0.0F));
        ImGui::PushStyleColor(
            ImGuiCol_Text,
            !alert.empty() ? RED
            : current      ? ORANGE
                           : ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled));
        if (ImGui::Button(info.name))
        {
            selectWorkspace(p_app, static_cast<Workspace>(i));
        }
        ImGui::PopStyleColor(2);
        if (alert.empty())
        {
            ImGui::SetItemTooltip("%s  (%s)", info.hint, info.shortcut);
        }
        else
        {
            ImGui::SetItemTooltip(
                "%s  (%s)\n\n%s", info.hint, info.shortcut, alert.c_str());
        }
    }
}

/// File, workspace and help menus.
static void mainMenu(App& p_app)
{
    if (!ImGui::BeginMainMenuBar())
    {
        return;
    }
    ImGui::TextColored(ORANGE, "ROBOTIK");
    ImGui::SameLine(0.0F, 20.0F);
    if (ImGui::BeginMenu("File"))
    {
        if (ImGui::MenuItem("Open scenario...", "Ctrl+O"))
        {
            p_app.ui.show_open = true;
        }
        if (ImGui::MenuItem(
                "Reload", "F5", false, !p_app.scenario_path.empty()))
        {
            reload(p_app);
        }
        ImGui::Separator();
        if (ImGui::MenuItem("Quit", "Ctrl+Q"))
        {
            quit();
        }
        ImGui::EndMenu();
    }
    drawDemosMenu(p_app);
    if (ImGui::BeginMenu("Windows"))
    {
        panelMenu(p_app);
        ImGui::EndMenu();
    }
    if (ImGui::BeginMenu("Layout"))
    {
        for (std::size_t i = 0; i < WORKSPACE_COUNT; ++i)
        {
            WorkspaceInfo const& info = WORKSPACES[i];
            if (ImGui::MenuItem(info.name,
                                info.shortcut,
                                p_app.ui.workspace ==
                                    static_cast<Workspace>(i)))
            {
                selectWorkspace(p_app, static_cast<Workspace>(i));
            }
        }
        ImGui::Separator();
        if (ImGui::MenuItem("Place open panels"))
            arrangeWorkspace(p_app, false);
        if (ImGui::MenuItem("Restore recommended layout"))
            arrangeWorkspace(p_app, true);
        ImGui::EndMenu();
    }
    drawPluginMenus(p_app);
    if (ImGui::BeginMenu("Help"))
    {
        if (ImGui::MenuItem("Keyboard shortcuts", "F1"))
        {
            p_app.ui.show_shortcuts = true;
        }
        ImGui::Separator();
        ImGui::TextDisabled("Documentation: doc/README.md");
        ImGui::TextDisabled("Tutorial: doc/tutorials/04-your-first-plugin.md");
        ImGui::EndMenu();
    }
    workspaceTabs(p_app);
    ImGui::EndMainMenuBar();
}

// --- Status bar -------------------------------------------------------------

/// Status-bar label and its color.
struct Outcome
{
    char const* text;
    ImVec4 color;
};

/// Status-bar text and color for the current run.
static Outcome outcomeOf(App const& p_app)
{
    if (!p_app.simulation)
    {
        return { p_app.error.empty() ? "NO SCENARIO" : "ERROR",
                 p_app.error.empty() ? GREY : RED };
    }
    robotik::Simulation const& simulation = *p_app.simulation;
    switch (simulation.outcome())
    {
        case robotik::Simulation::Outcome::Running:
            if (p_app.teach.manual)
            {
                return { "MANUAL", ORANGE };
            }
            return { p_app.playing ? "RUNNING" : "PAUSED",
                     p_app.playing ? ORANGE : BLUE };
        case robotik::Simulation::Outcome::ControllerSucceeded:
        case robotik::Simulation::Outcome::StopCondition:
            return { simulation.passed() ? "SUCCEEDED" : "ENDED",
                     simulation.passed() ? GREEN : RED };
        case robotik::Simulation::Outcome::ControllerFailed:
            return { "FAILED", RED };
        case robotik::Simulation::Outcome::Timeout:
            return { "TIMEOUT", RED };
        case robotik::Simulation::Outcome::AssertionFailed:
            return { "ASSERTION FAILED", RED };
        case robotik::Simulation::Outcome::Interrupted:
            return { "INTERRUPTED", GREY };
    }
    return { "", GREY };
}

/// Opens a diagnostic without changing the current layout preset.
static void showInMission(App& p_app, std::string p_window)
{
    for (std::size_t i = 0; i < PANEL_COUNT; ++i)
        if (PANELS[i].name == p_window)
        {
            showPanel(p_app, static_cast<PanelKind>(i));
            return;
        }
}

/// Clickable text in @p_color in a menu bar; true when clicked.
static bool barLink(char const* p_text, ImVec4 const& p_color)
{
    ImGui::PushStyleColor(ImGuiCol_Text, p_color);
    bool const clicked = ImGui::Selectable(
        p_text, false, 0, ImVec2(ImGui::CalcTextSize(p_text).x, 0.0F));
    ImGui::PopStyleColor();
    return clicked;
}

/// Bottom bar: scenario, seed, verdict, faults, selection; time and speed
/// on the right. Every count is a link to where it is explained.
static void statusBar(App& p_app)
{
    float const height = ImGui::GetFrameHeight() + 4.0F;
    ImGuiWindowFlags const flags = ImGuiWindowFlags_NoScrollbar |
                                   ImGuiWindowFlags_NoSavedSettings |
                                   ImGuiWindowFlags_MenuBar;
    if (!ImGui::BeginViewportSideBar(
            "##status", ImGui::GetMainViewport(), ImGuiDir_Down, height, flags))
    {
        ImGui::End();
        return;
    }
    if (ImGui::BeginMenuBar())
    {
        if (!p_app.simulation)
        {
            Outcome const outcome = outcomeOf(p_app);
            badge(outcome.text, outcome.color);
        }
        if (p_app.simulation)
        {
            robotik::Simulation const& simulation = *p_app.simulation;
            if (!p_app.scenario_title.empty())
            {
                ImGui::TextUnformatted(p_app.scenario_title.c_str());
            }
            ImGui::TextDisabled("seed %llu",
                                static_cast<unsigned long long>(p_app.seed));
            std::size_t passed = 0;
            std::size_t failed = 0;
            auto const& checks = simulation.checks();
            for (robotik::Check const& check : checks)
            {
                passed +=
                    check.verdict == robotik::Check::Verdict::Passed ? 1U : 0U;
                failed +=
                    check.verdict == robotik::Check::Verdict::Failed ? 1U : 0U;
            }
            char text[64];
            std::snprintf(text,
                          sizeof text,
                          "Assertions %zu/%zu%s",
                          passed,
                          checks.size(),
                          failed > 0U ? "  failed" : "");
            if (barLink(text,
                        failed > 0U               ? RED
                        : passed == checks.size() ? GREEN
                                                  : GREY))
            {
                showInMission(p_app, panelTitle("Assertions", "mission"));
            }
            ImGui::SetItemTooltip("Show the assertions.");

            robotik::ResourceManager const& resources =
                simulation.cell().robot().resources();
            std::size_t faults = 0;
            std::string first;
            for (robotik::ResourceId id = 0; id < resources.size(); ++id)
            {
                if (!resources.available(id))
                {
                    faults += 1U;
                    first = first.empty() ? resources.name(id) : first;
                }
            }
            if (faults > 0U)
            {
                std::snprintf(text, sizeof text, "%zu in fault", faults);
                if (barLink(text, RED))
                {
                    selectItem(p_app, SelectionKind::Resource, first);
                    showInspector(p_app);
                }
                ImGui::SetItemTooltip("Show the resource in the Inspector.");
            }
            if (emergencyEngaged(p_app))
            {
                badge("EMERGENCY STOP", RED);
            }
            Selection const& chosen = p_app.ui.selection;
            if (chosen.kind != SelectionKind::None)
            {
                ImGui::TextDisabled("|");
                std::snprintf(text, sizeof text, "%s", chosen.name.c_str());
                if (barLink(text, ImGui::GetStyleColorVec4(ImGuiCol_Text)))
                {
                    showInspector(p_app);
                }
                ImGui::SetItemTooltip("The selection. Show it in the "
                                      "Inspector.");
            }
        }
        if (!p_app.error.empty())
        {
            ImGui::PushStyleColor(ImGuiCol_Text, RED);
            ImGui::TextUnformatted(p_app.error.c_str());
            ImGui::PopStyleColor();
        }
        if (p_app.simulation)
        {
            char right[96];
            std::snprintf(right,
                          sizeof right,
                          "x%.2f real time   %.0f fps",
                          static_cast<double>(p_app.ui.realtime_factor),
                          static_cast<double>(ImGui::GetIO().Framerate));
            float const width = ImGui::CalcTextSize(right).x +
                                ImGui::GetStyle().WindowPadding.x;
            ImGui::SetCursorPosX(std::max(ImGui::GetCursorPosX(),
                                          ImGui::GetWindowWidth() - width));
            ImGui::TextDisabled("%s", right);
        }
        ImGui::EndMenuBar();
    }
    ImGui::End();
}

/// Run bar under the menu, in every workspace: play, step, the clock, the
/// outcome, replay and the seed, the speed; the emergency stop on the right.
static void runBar(App& p_app)
{
    float const height = ImGui::GetFrameHeight() + 14.0F;
    ImGui::PushStyleColor(ImGuiCol_WindowBg,
                          ImVec4(0.125F, 0.137F, 0.157F, 1.0F));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(12.0F, 7.0F));
    bool const open = ImGui::BeginViewportSideBar(
        "##run",
        ImGui::GetMainViewport(),
        ImGuiDir_Up,
        height,
        ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoSavedSettings |
            ImGuiWindowFlags_NoScrollWithMouse);
    ImGui::PopStyleVar();
    ImGui::PopStyleColor();
    if (!open)
    {
        ImGui::End();
        return;
    }
    if (!p_app.simulation)
    {
        ImGui::AlignTextToFramePadding();
        ImGui::TextDisabled("No scenario loaded: choose one in the Demos "
                            "menu, or File > Open scenario (Ctrl+O).");
        ImGui::End();
        return;
    }
    robotik::Simulation const& simulation = *p_app.simulation;

    ImGui::PushStyleColor(ImGuiCol_Button,
                          p_app.playing ? ImVec4(0.42F, 0.31F, 0.14F, 1.0F)
                                        : ImVec4(0.20F, 0.36F, 0.27F, 1.0F));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered,
                          p_app.playing ? ImVec4(0.52F, 0.39F, 0.18F, 1.0F)
                                        : ImVec4(0.25F, 0.45F, 0.33F, 1.0F));
    if (ImGui::Button(p_app.playing ? "Pause" : "Play", ImVec2(72.0F, 0.0F)))
    {
        p_app.playing = !p_app.playing;
    }
    ImGui::PopStyleColor(2);
    ImGui::SetItemTooltip("Space");
    ImGui::SameLine();
    ImGui::BeginDisabled(p_app.playing);
    if (ImGui::Button("Step"))
    {
        p_app.step_once = true;
    }
    ImGui::EndDisabled();
    ImGui::SetItemTooltip("One step of 10 ms  (.)");

    // The clock, large, then what the run is doing.
    ImGui::SameLine(0.0F, 18.0F);
    ImGui::AlignTextToFramePadding();
    ImGui::PushFont(monoFont(), ImGui::GetStyle().FontSizeBase * 1.15F);
    ImGui::TextColored(ImVec4(0.93F, 0.71F, 0.39F, 1.0F),
                       "%8.2f s",
                       simulation.time().value());
    ImGui::PopFont();
    ImGui::SameLine(0.0F, 12.0F);
    Outcome const outcome = outcomeOf(p_app);
    ImGui::SetCursorPosY(
        ImGui::GetCursorPosY() +
        (ImGui::GetFrameHeight() - ImGui::GetTextLineHeight() - 2.0F) * 0.5F);
    badge(outcome.text, outcome.color);

    ImGui::SameLine(0.0F, 28.0F);
    if (ImGui::Button("Replay"))
    {
        p_app.reset(p_app.seed);
    }
    ImGui::SetItemTooltip("Start again with this seed and play  (Ctrl+R)");
    ImGui::SameLine(0.0F, 12.0F);
    char speed_label[24];
    std::snprintf(speed_label,
                  sizeof speed_label,
                  "x%.2g",
                  static_cast<double>(p_app.speed));
    ImGui::SetNextItemWidth(84.0F);
    if (ImGui::BeginCombo("##run_speed", speed_label))
    {
        static constexpr float SPEEDS[] = { 0.25F, 0.5F, 1.0F, 2.0F, 4.0F };
        for (float const speed : SPEEDS)
        {
            char label[24];
            std::snprintf(
                label, sizeof label, "x%.2g", static_cast<double>(speed));
            if (ImGui::Selectable(label, p_app.speed == speed))
                p_app.speed = speed;
        }
        ImGui::EndCombo();
    }
    ImGui::SetItemTooltip(
        "Simulation speed. Run options offers a continuous setting.");
    ImGui::SameLine();
    if (ImGui::Button("Run options..."))
        ImGui::OpenPopup("run_options");
    if (ImGui::BeginPopup("run_options"))
    {
        ImGui::SeparatorText("Replay");
        ImGui::SetNextItemWidth(150.0F);
        ImGui::InputScalar("Seed", ImGuiDataType_U64, &p_app.seed);
        ImGui::SetItemTooltip("Seed for the next replay: object layout, noise "
                              "and random faults.");
        if (ImGui::Button("Next seed and replay"))
        {
            p_app.reset(p_app.seed + 1U);
            ImGui::CloseCurrentPopup();
        }
        ImGui::SetItemTooltip("Ctrl+N");
        ImGui::SeparatorText("Speed");
        ImGui::SetNextItemWidth(210.0F);
        ImGui::SliderFloat(
            "##continuous_speed", &p_app.speed, 0.1F, 4.0F, "x%.2f");
        ImGui::EndPopup();
    }
    if (p_app.owns_clock)
    {
        ImGui::SameLine();
        ImGui::AlignTextToFramePadding();
        badge("Plugin clock", BLUE);
        ImGui::SetItemTooltip("The plugin advances simulated time.");
    }

    if (emergencyAvailable(p_app))
    {
        bool const engaged = emergencyEngaged(p_app);
        char const* label = engaged ? "Release stop" : "EMERGENCY STOP";
        float const width = ImGui::CalcTextSize(label).x + 28.0F;
        ImGui::SameLine();
        ImGui::SetCursorPosX(std::max(ImGui::GetCursorPosX(),
                                      ImGui::GetWindowWidth() - width -
                                          ImGui::GetStyle().WindowPadding.x));
        ImGui::PushStyleColor(ImGuiCol_Button,
                              engaged ? ImVec4(0.30F, 0.30F, 0.32F, 1.0F)
                                      : ImVec4(0.70F, 0.16F, 0.16F, 1.0F));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered,
                              engaged ? ImVec4(0.40F, 0.40F, 0.42F, 1.0F)
                                      : ImVec4(0.85F, 0.22F, 0.22F, 1.0F));
        if (ImGui::Button(label, ImVec2(width, 0.0F)))
        {
            toggleEmergency(p_app);
        }
        ImGui::PopStyleColor(2);
        ImGui::SetItemTooltip("The Stop skill takes every actuator  (Ctrl+E)");
    }
    ImGui::End();
}

// --- Dialogs ----------------------------------------------------------------

/// Modal that asks for a scenario path.
static void openDialog(App& p_app)
{
    if (p_app.ui.show_open)
    {
        p_app.ui.show_open = false;
        p_app.ui.open_path = p_app.scenario_path.string();
        ImGui::OpenPopup("Open scenario");
    }
    ImGui::SetNextWindowSize(ImVec2(620.0F, 0.0F), ImGuiCond_Appearing);
    if (!ImGui::BeginPopupModal(
            "Open scenario", nullptr, ImGuiWindowFlags_AlwaysAutoResize))
    {
        return;
    }
    ImGui::TextUnformatted("Scenario file (.scenario.yaml), listed in the "
                           "plugin.yaml of its package:");
    ImGui::SetNextItemWidth(600.0F);
    if (ImGui::IsWindowAppearing())
    {
        ImGui::SetKeyboardFocusHere();
    }
    bool const enter = ImGui::InputText(
        "##path", &p_app.ui.open_path, ImGuiInputTextFlags_EnterReturnsTrue);
    ImGui::TextDisabled(
        "Tip: the Demos menu lists every scenario of the built plugins.");
    if (ImGui::Button("Open") || enter)
    {
        requestScenario(p_app, std::filesystem::path(p_app.ui.open_path));
        ImGui::CloseCurrentPopup();
    }
    ImGui::SameLine();
    if (ImGui::Button("Cancel") || ImGui::IsKeyPressed(ImGuiKey_Escape))
    {
        ImGui::CloseCurrentPopup();
    }
    ImGui::EndPopup();
}

/// Lists the keyboard shortcuts.
static void shortcutsWindow(App& p_app)
{
    if (!p_app.ui.show_shortcuts)
    {
        return;
    }
    ImGui::SetNextWindowSize(ImVec2(420.0F, 0.0F), ImGuiCond_Appearing);
    ImGui::SetNextWindowPos(ImGui::GetMainViewport()->GetCenter(),
                            ImGuiCond_Appearing,
                            ImVec2(0.5F, 0.5F));
    if (ImGui::Begin("Keyboard shortcuts",
                     &p_app.ui.show_shortcuts,
                     ImGuiWindowFlags_NoDocking | ImGuiWindowFlags_NoCollapse |
                         ImGuiWindowFlags_AlwaysAutoResize))
    {
        static constexpr char const* ROWS[][2] = {
            { "Space", "Play / pause" },
            { ".", "One step (when paused)" },
            { "Ctrl+R", "Replay with the same seed" },
            { "Ctrl+N", "Replay with the next seed" },
            { "Ctrl+E", "Emergency stop / release" },
            { "Ctrl+Space",
              "Expand the focused window; Esc restores the layout" },
            { "F5", "Reload the scenario file" },
            { "Ctrl+O", "Open a scenario file" },
            { "Ctrl+1 ... Ctrl+4", "Plugin, Robot, Mission, Autonomy" },
            { "H", "Sent to the plugin (Home in most demos)" },
            { "Left drag", "Drag the ground of the 3D view" },
            { "Right or middle drag", "Turn the 3D view" },
            { "Wheel, R / F", "Zoom the 3D view" },
            { "W A S D, arrows", "Move the 3D view (over the view)" },
            { "Q / E", "Turn the 3D view (over the view)" },
            { "Shift", "Faster" },
            { "F1", "This window" },
        };
        if (beginTable("keys",
                       2,
                       ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerV))
        {
            for (auto const& row : ROWS)
            {
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::TextColored(BLUE, "%s", row[0]);
                ImGui::TableNextColumn();
                ImGui::TextUnformatted(row[1]);
            }
            endTable();
        }
    }
    ImGui::End();
}

/// Brings the tabs chosen by the layout to the front. ImGui selects each
/// new tab as it appears, and the windows of a layout get their tab over
/// several frames: this is done again for about a second, and stops at the
/// first click so it never fights the operator.
static void frontTabs(UiState& p_ui)
{
    if (p_ui.front_tabs.empty())
        return;
    if (ImGui::IsAnyMouseDown())
    {
        p_ui.front_tabs.clear();
        return;
    }
    // ImGui focuses the last window that appeared and shows the tab of the
    // focused window: no focus until the operator picks one (an editor
    // asked for is focused again at the end of the frame).
    ImGui::SetWindowFocus(nullptr);
    for (std::string const& name : p_ui.front_tabs)
    {
        ImGuiWindow const* window = ImGui::FindWindowByName(name.c_str());
        if (window == nullptr || window->DockNode == nullptr ||
            window->DockNode->TabBar == nullptr)
            continue;
        ImGuiTabBar* bar = window->DockNode->TabBar;
        if (ImGui::TabBarFindTabByID(bar, window->TabId) != nullptr)
            bar->NextSelectedTabId = window->TabId;
    }
    if (++p_ui.front_frames >= 60)
        p_ui.front_tabs.clear();
}

void drawPanels(App& p_app)
{
    // A scenario may replace the plugin that owned the expanded editor.
    if (!p_app.ui.maximized_window.empty())
    {
        bool const standard = std::ranges::any_of(
            PANELS,
            [&](PanelInfo const& p_panel)
            { return p_app.ui.maximized_window == p_panel.name; });
        bool const plugin =
            std::ranges::any_of(p_app.plugins.host().panels(),
                                [&](robotik::PluginPanel const& p_panel)
                                {
                                    return p_app.ui.maximized_window ==
                                           pluginWindowTitle(p_app, p_panel);
                                });
        if (!standard && !plugin)
            p_app.ui.pending_maximized = std::string();
    }
    if (!p_app.ui.pending_scenario.empty())
    {
        p_app.scenario_path = p_app.ui.pending_scenario;
        p_app.ui.pending_scenario.clear();
        p_app.load();
    }
    openClickedAsset(p_app);
    shortcuts(p_app);
    mainMenu(p_app);
    runBar(p_app);
    statusBar(p_app);

    Workspace const workspace = p_app.ui.workspace;
    WorkspaceInfo const& info = infoOf(workspace);
    // LAYOUT_VERSION (top of the file) names the dock space: a new version
    // finds no saved node and builds the default layout.
    ImGuiID const dock = ImGui::GetID(
        ("RobotikEditors/" + std::to_string(LAYOUT_VERSION)).c_str());
    ImGuiDockNode const* node = ImGui::DockBuilderGetNode(dock);
    bool const first = !p_app.ui.layout_checked;
    p_app.ui.layout_checked = true;
    if (p_app.ui.maximized_window.empty() &&
        (p_app.ui.reset_layout || node == nullptr))
    {
        bool const graph = p_app.simulation &&
                           (p_app.simulation->cell().planner() != nullptr ||
                            (p_app.simulation->cell().tree() != nullptr &&
                             p_app.simulation->cell().tree()->hasRoot()));
        // Mission follows the scenario: the Planner when it plans (the
        // Windows menu opens the others).
        if (workspace == Workspace::Mission && p_app.simulation)
        {
            if (p_app.simulation->cell().planner() != nullptr)
                p_app.ui.panels[static_cast<std::size_t>(PanelKind::Planner)] =
                    true;
        }
        p_app.ui.front_tabs.clear();
        p_app.ui.front_frames = 0;
        layout(workspace, dock, graph, p_app.ui.front_tabs);
        if (workspace == Workspace::Mission && p_app.simulation &&
            p_app.simulation->cell().planner() != nullptr)
            front(p_app.ui.front_tabs, "Planner");
        // An editor asked for (Windows menu, a link) wins over the defaults:
        // the last one named in a tab group is shown.
        p_app.ui.front_tabs.insert(p_app.ui.front_tabs.end(),
                                   p_app.ui.focus_windows.begin(),
                                   p_app.ui.focus_windows.end());
        if (workspace == Workspace::Robot && p_app.simulation &&
            p_app.ui.selection.kind == SelectionKind::None &&
            !p_app.simulation->cell().robots().empty())
            p_app.ui.selection = {
                SelectionKind::Robot,
                p_app.simulation->cell().robots().front()->id()
            };
        p_app.ui.reset_layout = false;
    }
    // One way to close a window: the cross of its tab (or its right-click
    // menu). The dock menu and the second cross of the node are hidden.
    ImGuiDockNodeFlags const chrome = ImGuiDockNodeFlags_NoWindowMenuButton |
                                      ImGuiDockNodeFlags_NoCloseButton;
    ImGui::DockSpaceOverViewport(dock,
                                 ImGui::GetMainViewport(),
                                 p_app.ui.maximized_window.empty()
                                     ? chrome
                                     : ImGuiDockNodeFlags_KeepAliveOnly);
    p_app.view_hovered = false;
    // Set again by the Asset tab of Files when it is drawn.
    p_app.ui.asset_preview.visible = false;
    if (p_app.ui.asset_preview.model)
    {
        p_app.ui.asset_preview.model->area.hovered = false;
    }
    sampleTimeline(p_app);
    if (p_app.ui.maximized_window.empty())
        frontTabs(p_app.ui);
    for (std::size_t i = 0; i < PANEL_COUNT; ++i)
        if (p_app.ui.panels[i] && (p_app.ui.maximized_window.empty() ||
                                   p_app.ui.maximized_window == PANELS[i].name))
            PANELS[i].draw(p_app);

    syncTeachAxes(
        p_app,
        p_app.teach.manual &&
            p_app.ui.panels[static_cast<std::size_t>(PanelKind::Teach)]);
    ImGuiID target = 0;
    if (p_app.ui.place_plugin_panels && p_app.ui.maximized_window.empty())
    {
        ImGuiWindow const* anchor = ImGui::FindWindowByName(info.anchor);
        if (anchor != nullptr && anchor->DockId != 0)
        {
            target = anchor->DockId;
            p_app.ui.place_plugin_panels = false;
        }
    }
    bool const focus = p_app.ui.focus_plugin_panel && target != 0;
    if (focus)
        p_app.ui.focus_plugin_panel = false;
    drawPluginPanels(p_app, target, focus);
    if (p_app.ui.maximized_window.empty())
    {
        for (std::string const& window : p_app.ui.focus_windows)
            ImGui::SetWindowFocus(window.c_str());
        p_app.ui.focus_windows.clear();
    }
    sceneInset(p_app);
    drawCorners(p_app);
    if (first && p_app.ui.focused_window.empty())
        p_app.ui.focused_window = "3D scene";
    openDialog(p_app);
    shortcutsWindow(p_app);
}
