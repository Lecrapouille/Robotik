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

#include <array>
#include <cstdio>
#include <filesystem>
#include <string>

/// Title, shortcut and hint of one workspace.
struct WorkspaceInfo
{
    char const* name;
    char const* key;
    char const* shortcut;
    /// A panel of the column where the plugin panels go.
    char const* anchor;
    char const* hint;
};

static std::array<WorkspaceInfo, WORKSPACE_COUNT> const WORKSPACES{ {
    { "Plugin",
      "plugin",
      "Ctrl+1",
      "Files",
      "Read the loaded plugin: its files, dependencies and licenses, the "
      "scenario, "
      "its behavior tree, the experiments, the world and the asset packs. No "
      "3D view." },
    { "Robot",
      "robot",
      "Ctrl+2",
      "Teach Pendant",
      "The robot body: joints and their limits, plots, manual moves, "
      "cameras." },
    { "Mission",
      "mission",
      "Ctrl+3",
      "Skills",
      "The execution: behavior tree, skills, resources and their timeline, the "
      "PDDL planner." },
    { "Autonomy",
      "autonomy",
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

/// Switches the workspace and asks for the plugin panels to be placed.
static void select(App& p_app, Workspace p_workspace)
{
    if (p_app.ui.workspace != p_workspace)
    {
        p_app.ui.workspace = p_workspace;
        p_app.ui.place_plugin_panels = true;
    }
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

/// Builds the dock layout of one workspace.
static void layout(Workspace p_workspace, ImGuiID p_dock)
{
    ImGui::DockBuilderRemoveNode(p_dock);
    ImGui::DockBuilderAddNode(p_dock, ImGuiDockNodeFlags_DockSpace);
    ImGui::DockBuilderSetNodeSize(p_dock, ImGui::GetMainViewport()->WorkSize);
    char const* key = infoOf(p_workspace).key;
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
            dock("###View", right);
            dock("Assets", assets);
            break;
        }
        case Workspace::Robot:
        {
            ImGuiID const left = split(center, ImGuiDir_Left, 0.36F);
            ImGuiID right = split(center, ImGuiDir_Right, 0.28F);
            ImGuiID const cameras = split(right, ImGuiDir_Down, 0.42F);
            ImGuiID const bottom = split(center, ImGuiDir_Down, 0.40F);
            dock("Joints", left);
            dock(mainWindowId(key), center);
            dock("Joint plots", bottom);
            dock(panelTitle("Inspector", key), right);
            dock("Teach Pendant", right);
            dock(panelTitle("Cameras", key), cameras);
            break;
        }
        case Workspace::Mission:
        {
            // The Timeline takes the whole width: a time axis needs it.
            // Above it the scene, between the tree (with the verdict below)
            // and the skills.
            ImGuiID const bottom = split(center, ImGuiDir_Down, 0.30F);
            ImGuiID left = split(center, ImGuiDir_Left, 0.20F);
            ImGuiID const verdict = split(left, ImGuiDir_Down, 0.40F);
            ImGuiID inspector = split(center, ImGuiDir_Right, 0.30F);
            ImGuiID const right = split(inspector, ImGuiDir_Down, 0.45F);
            dock(panelTitle("Behavior tree", key), left);
            dock(panelTitle("Assertions", key), verdict);
            dock(mainWindowId(key), center);
            dock("Timeline", bottom);
            dock(panelTitle("Inspector", key), inspector);
            // The last one docked is in front: Skills.
            dock("Planner", right);
            dock("Resources", right);
            dock("Skills", right);
            break;
        }
        case Workspace::Autonomy:
        {
            ImGuiID const left = split(center, ImGuiDir_Left, 0.30F);
            ImGuiID right = split(center, ImGuiDir_Right, 0.32F);
            ImGuiID const perception = split(right, ImGuiDir_Down, 0.45F);
            dock("World model", left);
            dock(mainWindowId(key), center);
            dock(panelTitle("Cameras", key), right);
            dock(panelTitle("Inspector", key), right);
            dock("Perception", perception);
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

void toggleMaximized(App& p_app, ViewKind p_view)
{
    p_app.ui.maximized =
        p_app.ui.maximized == p_view ? ViewKind::None : p_view;
}

void drawView(App& p_app, ViewKind p_view)
{
    switch (p_view)
    {
        case ViewKind::Scene:
            sceneView(p_app);
            break;
        case ViewKind::Camera:
            cameraView(p_app);
            break;
        case ViewKind::Timeline:
            timelineView(p_app);
            break;
        case ViewKind::Plots:
            plotsView(p_app);
            break;
        case ViewKind::Tree:
            treeView(p_app);
            break;
        case ViewKind::Planner:
            plannerView(p_app);
            break;
        case ViewKind::None:
            break;
    }
}

char const* viewName(ViewKind p_view)
{
    switch (p_view)
    {
        case ViewKind::Scene:
            return "3D scene";
        case ViewKind::Camera:
            return "Camera";
        case ViewKind::Timeline:
            return "Timeline";
        case ViewKind::Plots:
            return "Joint plots";
        case ViewKind::Tree:
            return "Behavior tree";
        case ViewKind::Planner:
            return "Planner";
        case ViewKind::None:
            break;
    }
    return "";
}

/// The view in the main area of the workspace shown; None in Plugin.
static ViewKind mainViewOf(App const& p_app)
{
    if (p_app.ui.workspace == Workspace::Plugin)
    {
        return ViewKind::None;
    }
    return p_app.ui.main_view[static_cast<std::size_t>(p_app.ui.workspace)];
}

bool viewElsewhere(App const& p_app, ViewKind p_view)
{
    return p_app.ui.maximized == p_view || mainViewOf(p_app) == p_view;
}

void viewNote(App& p_app, ViewKind p_view)
{
    bool const full = p_app.ui.maximized == p_view;
    ImGui::TextDisabled("%s", full ? "Shown over the whole work area."
                                   : "Shown in the main area.");
    if (ImGui::SmallButton("Show it here"))
    {
        if (full)
        {
            p_app.ui.maximized = ViewKind::None;
        }
        else
        {
            p_app.ui.main_view[static_cast<std::size_t>(p_app.ui.workspace)] =
                ViewKind::Scene;
        }
    }
    ImGui::SetItemTooltip(full ? "Back to the panels (Ctrl+Space or Esc)."
                               : "The main area shows the 3D scene again.");
}

/// The maximized view over the whole work area, between the run bar and the
/// status bar, with a line to come back.
static void maximizedWindow(App& p_app)
{
    UiState& ui = p_app.ui;
    if (!p_app.simulation || ui.workspace == Workspace::Plugin)
    {
        ui.maximized = ViewKind::None;
    }
    if (ui.maximized == ViewKind::None)
    {
        ui.maximized_last = ViewKind::None;
        return;
    }
    ImGuiViewport const* viewport = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(viewport->WorkPos);
    ImGui::SetNextWindowSize(viewport->WorkSize);
    ImGui::SetNextWindowViewport(viewport->ID);
    if (ui.maximized != ui.maximized_last)
    {
        ImGui::SetNextWindowFocus();
    }
    ui.maximized_last = ui.maximized;
    ImGuiWindowFlags const flags =
        ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoDocking |
        ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoMove |
        ImGuiWindowFlags_NoScrollWithMouse;
    if (ImGui::Begin("##maximized", nullptr, flags))
    {
        ImGui::AlignTextToFramePadding();
        ImGui::TextUnformatted(viewName(ui.maximized));
        ImGui::SameLine();
        ImGui::TextDisabled("Ctrl+Space or Esc: back to the panels");
        ImGui::SameLine();
        float const width = ImGui::CalcTextSize("Restore").x +
                            2.0F * ImGui::GetStyle().FramePadding.x;
        ImGui::SetCursorPosX(std::max(ImGui::GetCursorPosX(),
                                      ImGui::GetWindowWidth() - width -
                                          ImGui::GetStyle().WindowPadding.x));
        if (ImGui::Button("Restore"))
        {
            ui.maximized = ViewKind::None;
        }
        bool const scene = ui.maximized == ViewKind::Scene;
        if (scene)
        {
            ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0F, 0.0F));
        }
        if (ImGui::BeginChild("view",
                              ImVec2(0.0F, 0.0F),
                              ImGuiChildFlags_None,
                              scene ? ImGuiWindowFlags_NoScrollbar |
                                          ImGuiWindowFlags_NoScrollWithMouse
                                    : ImGuiWindowFlags_None))
        {
            drawView(p_app, ui.maximized);
        }
        ImGui::EndChild();
        if (scene)
        {
            ImGui::PopStyleVar();
        }
    }
    ImGui::End();
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
            select(p_app, static_cast<Workspace>(i));
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
    // The view under the mouse over the whole work area, or back.
    if (ctrl && ImGui::IsKeyPressed(ImGuiKey_Space, false))
    {
        ViewKind const under = p_app.ui.maximized != ViewKind::None
                                    ? p_app.ui.maximized
                                : p_app.ui.hovered_view != ViewKind::None
                                    ? p_app.ui.hovered_view
                                    : ViewKind::Scene;
        toggleMaximized(p_app, under);
    }
    if (p_app.ui.maximized != ViewKind::None &&
        ImGui::IsKeyPressed(ImGuiKey_Escape, false))
    {
        p_app.ui.maximized = ViewKind::None;
    }
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
        ImGui::PushStyleColor(
            ImGuiCol_Button,
            current ? ImGui::GetStyleColorVec4(ImGuiCol_ButtonActive)
                    : ImVec4(0.0F, 0.0F, 0.0F, 0.0F));
        ImGui::PushStyleColor(
            ImGuiCol_Text,
            !alert.empty() ? RED
            : current      ? ImGui::GetStyleColorVec4(ImGuiCol_Text)
                           : ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled));
        if (ImGui::Button(info.name))
        {
            select(p_app, static_cast<Workspace>(i));
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
    if (ImGui::BeginMenu("View"))
    {
        for (std::size_t i = 0; i < WORKSPACE_COUNT; ++i)
        {
            WorkspaceInfo const& info = WORKSPACES[i];
            if (ImGui::MenuItem(info.name,
                                info.shortcut,
                                p_app.ui.workspace ==
                                    static_cast<Workspace>(i)))
            {
                select(p_app, static_cast<Workspace>(i));
            }
        }
        ImGui::Separator();
        if (ImGui::MenuItem("Reset layout"))
        {
            p_app.ui.reset_layout = true;
            p_app.ui.place_plugin_panels = true;
        }
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
        ImGui::TextDisabled("Tutorial: doc/04-Your-First-Plugin.md");
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

/// Shows @p_window of the Mission workspace in front.
static void showInMission(App& p_app, std::string p_window)
{
    select(p_app, Workspace::Mission);
    p_app.ui.focus_window = std::move(p_window);
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
            ImGui::SetItemTooltip("Show the assertions (Mission workspace).");

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
    ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.125F, 0.137F, 0.157F, 1.0F));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(12.0F, 7.0F));
    bool const open = ImGui::BeginViewportSideBar(
        "##run", ImGui::GetMainViewport(), ImGuiDir_Up, height,
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
    ImGui::SetCursorPosY(ImGui::GetCursorPosY() +
                         (ImGui::GetFrameHeight() -
                          ImGui::GetTextLineHeight() - 2.0F) *
                             0.5F);
    badge(outcome.text, outcome.color);

    ImGui::SameLine(0.0F, 28.0F);
    if (ImGui::Button("Replay"))
    {
        p_app.reset(p_app.seed);
    }
    ImGui::SetItemTooltip("Start again with this seed and play  (Ctrl+R)");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(90.0F);
    ImGui::InputScalar("##seed", ImGuiDataType_U64, &p_app.seed);
    ImGui::SetItemTooltip(
        "Seed: object layout, sensor noise and random faults derive from it.\n"
        "Press Replay to use it.");
    ImGui::SameLine();
    if (ImGui::Button("Next seed"))
    {
        p_app.reset(p_app.seed + 1U);
    }
    ImGui::SetItemTooltip("Ctrl+N");
    ImGui::SameLine(0.0F, 18.0F);
    ImGui::SetNextItemWidth(130.0F);
    ImGui::SliderFloat("##speed", &p_app.speed, 0.1F, 4.0F, "speed x%.1f");
    if (p_app.owns_clock)
    {
        ImGui::SameLine();
        ImGui::AlignTextToFramePadding();
        ImGui::TextDisabled("(the plugin drives the clock)");
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
            { "Ctrl+Space", "The view under the mouse over the whole work "
                            "area, or back (Esc)" },
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

void drawPanels(App& p_app)
{
    if (!p_app.ui.pending_scenario.empty())
    {
        p_app.scenario_path = p_app.ui.pending_scenario;
        p_app.ui.pending_scenario.clear();
        p_app.load();
    }
    openClickedAsset(p_app);
    shortcuts(p_app);
    // Set again by the views under the mouse in this frame.
    p_app.ui.hovered_view = ViewKind::None;
    mainMenu(p_app);
    runBar(p_app);
    statusBar(p_app);

    Workspace const workspace = p_app.ui.workspace;
    WorkspaceInfo const& info = infoOf(workspace);
    // Bump LAYOUT_VERSION when a default layout changes: the layouts saved in
    // the ini file are then rebuilt once.
    constexpr char const* LAYOUT_VERSION = "/11";
    ImGuiID const dock = ImGui::GetID(
        (std::string("RobotikWorkspace/") + info.key + LAYOUT_VERSION).c_str());
    // Every default layout is split: a whole node when the workspace is
    // first shown is a layout saved half built (the program stopped while
    // building it). It is built again; later, the operator may merge nodes.
    ImGuiDockNode const* node = ImGui::DockBuilderGetNode(dock);
    bool& checked = p_app.ui.layout_checked[static_cast<std::size_t>(workspace)];
    bool const broken = !checked && node != nullptr && !node->IsSplitNode();
    checked = true;
    if (p_app.ui.reset_layout || node == nullptr || broken)
    {
        layout(workspace, dock);
        p_app.ui.reset_layout = false;
        // Docking does not choose the tab in front.
        switch (workspace)
        {
            case Workspace::Robot:
                p_app.ui.focus_window = "Teach Pendant";
                break;
            case Workspace::Mission:
                p_app.ui.focus_window = "Skills";
                break;
            case Workspace::Autonomy:
                p_app.ui.focus_window = panelTitle("Inspector", info.key);
                break;
            case Workspace::Plugin:
                break;
        }
    }
    ImGui::DockSpaceOverViewport(dock, ImGui::GetMainViewport());

    if (workspace != Workspace::Plugin)
    {
        mainPanel(p_app, info.key);
    }
    // Set again by the Asset tab of Files when it is drawn.
    p_app.ui.asset_preview.visible = false;
    if (p_app.ui.asset_preview.model)
    {
        p_app.ui.asset_preview.model->area.hovered = false;
    }
    sampleTimeline(p_app);
    switch (workspace)
    {
        case Workspace::Plugin:
            pluginPanel(p_app);
            filesPanel(p_app);
            viewPanel(p_app);
            assetsPanel(p_app);
            break;
        case Workspace::Robot:
            robotPanel(p_app);
            plotsPanel(p_app);
            teachPanel(p_app);
            camerasPanel(p_app, info.key);
            inspectorPanel(p_app, info.key);
            break;
        case Workspace::Mission:
            treePanel(p_app, info.key);
            timelinePanel(p_app);
            inspectorPanel(p_app, info.key);
            resourcesPanel(p_app);
            assertionsPanel(p_app, info.key);
            skillsPanel(p_app);
            plannerPanel(p_app);
            break;
        case Workspace::Autonomy:
            worldModelPanel(p_app);
            camerasPanel(p_app, info.key);
            inspectorPanel(p_app, info.key);
            perceptionPanel(p_app);
            break;
    }

    maximizedWindow(p_app);
    syncTeachAxes(p_app, workspace == Workspace::Robot);
    if (!p_app.ui.focus_window.empty())
    {
        ImGui::SetWindowFocus(p_app.ui.focus_window.c_str());
        p_app.ui.focus_window.clear();
    }

    // Plugin panels follow the workspace: they join its side column. The
    // Plugin workspace reads the package and shows none: they wait there
    // for the next workspace.
    ImGuiID target = 0;
    if (workspace == Workspace::Plugin)
    {
        p_app.ui.focus_plugin_panel = false;
    }
    else if (p_app.ui.place_plugin_panels)
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
    {
        p_app.ui.focus_plugin_panel = false;
    }
    if (workspace != Workspace::Plugin)
    {
        drawPluginPanels(p_app, target, focus);
    }

    openDialog(p_app);
    shortcutsWindow(p_app);
}
