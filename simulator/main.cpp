// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "App.hpp"
#include "MainLoop.hpp"
#include "Ui.hpp"
#include "Window.hpp"

#include <filesystem>
#include <iostream>
#include <string>

/// The view named on the command line; the 3D scene for an unknown name.
static PanelKind viewNamed(std::string const& p_name)
{
    return p_name == "camera"      ? PanelKind::Cameras
           : p_name == "timeline"  ? PanelKind::Timeline
           : p_name == "plots"     ? PanelKind::Plots
           : p_name == "tree"      ? PanelKind::Graph
           : p_name == "planner"   ? PanelKind::Planner
           : p_name == "graph"     ? PanelKind::Graph
           : p_name == "events"    ? PanelKind::Events
           : p_name == "inspector" ? PanelKind::Inspector
                                   : PanelKind::Scene;
}

int main(int argc, char** argv)
{
    Window window;
    if (!window.open())
    {
        std::cerr << "Cannot open an OpenGL 4.5 window\n";
        return 1;
    }

    App app;
    app.plugins.scan();
    // Robotik-Simulator [scenario] [--screenshot file.png] [--screenshot-ui
    // file.png] [--at seconds] [--workspace plugin|robot|mission|autonomy]
    // [--select skill:<name>|resource:<name>|robot:<id>|object:<name>]
    // [--maximize <view>] [--view <view>], view: scene|camera|timeline|
    // plots|tree|planner|graph|events (--view opens and focuses the editor)
    std::string select;
    std::optional<Workspace> preset;
    std::optional<PanelKind> view;
    std::optional<PanelKind> maximized;
    bool tree_requested = false;
    for (int i = 1; i < argc; ++i)
    {
        std::string const argument = argv[i];
        if (argument == "--screenshot" && i + 1 < argc)
        {
            app.screenshot = argv[++i];
        }
        else if (argument == "--screenshot-ui" && i + 1 < argc)
        {
            app.screenshot_ui = argv[++i];
        }
        else if (argument == "--maximize" && i + 1 < argc)
        {
            std::string const name = argv[++i];
            maximized = viewNamed(name);
            tree_requested = name == "tree";
        }
        else if (argument == "--view" && i + 1 < argc)
        {
            std::string const name = argv[++i];
            view = viewNamed(name);
            tree_requested = name == "tree";
        }
        else if (argument == "--select" && i + 1 < argc)
        {
            select = argv[++i];
        }
        else if (argument == "--workspace" && i + 1 < argc)
        {
            std::string const name = argv[++i];
            app.ui.workspace = name == "plugin"     ? Workspace::Plugin
                               : name == "robot"    ? Workspace::Robot
                               : name == "autonomy" ? Workspace::Autonomy
                                                    : Workspace::Mission;
            preset = app.ui.workspace;
        }
        else if (argument == "--at" && i + 1 < argc)
        {
            app.screenshot_at = std::stod(argv[++i]);
        }
        else
        {
            app.scenario_path = std::filesystem::path(argument);
        }
    }
#if !defined(__EMSCRIPTEN__) || defined(EXAEQUOS)
    // The browser page opens on the plugin list. A scenario given on the
    // command line is still loaded, below.
    if (app.scenario_path.empty())
    {
        for (robotik::PluginPackage const& package :
             app.plugins.catalog().packages())
        {
            for (robotik::PluginScenario const& scenario : package.scenarios)
            {
                if (scenario.id == "pick_and_place")
                {
                    app.scenario_path = scenario.file;
                }
            }
        }
    }
#endif
    if (!app.scenario_path.empty())
    {
        app.load();
        // --select: what the Inspector shows first (load clears it).
        if (std::size_t const colon = select.find(':');
            colon != std::string::npos)
        {
            std::string const kind = select.substr(0, colon);
            app.ui.selection = { kind == "skill"      ? SelectionKind::Skill
                                 : kind == "resource" ? SelectionKind::Resource
                                 : kind == "object"   ? SelectionKind::Object
                                                      : SelectionKind::Robot,
                                 select.substr(colon + 1) };
        }
        // A picture of the run: play it until the time of the picture.
        app.playing =
            app.playing ||
            ((!app.screenshot.empty() || !app.screenshot_ui.empty()) &&
             app.screenshot_at > 0.0);
    }
    // Command-line choices take precedence over saved visibility and preset.
    initializePanelSettings(app);
    if (ImGui::GetIO().IniFilename != nullptr)
        ImGui::LoadIniSettingsFromDisk(ImGui::GetIO().IniFilename);
    // A workspace argument is an explicit request for its default editors.
    if (preset)
    {
        app.ui.workspace = *preset;
        app.ui.panels = defaultPanels(*preset);
        app.ui.reset_layout = true;
    }
    if (!app.screenshot.empty())
        showPanel(app, PanelKind::Scene);
    app.ui.graph_tree = tree_requested;
    if (view)
        showPanel(app, *view);
    if (maximized)
    {
        showPanel(app, *maximized);
        toggleMaximized(app, panelName(*maximized));
    }
    runMainLoop(window, app);
    return app.error.empty() ? 0 : 1;
}
