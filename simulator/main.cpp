// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "App.hpp"
#include "MainLoop.hpp"
#include "Window.hpp"

#include <filesystem>
#include <iostream>
#include <string>

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
    // [--maximize scene|timeline|camera]
    std::string select;
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
            app.ui.maximized = name == "timeline" ? Maximized::Timeline
                               : name == "camera" ? Maximized::Camera
                                                  : Maximized::Scene;
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
            app.ui.selection = {
                kind == "skill"      ? SelectionKind::Skill
                : kind == "resource" ? SelectionKind::Resource
                : kind == "object"   ? SelectionKind::Object
                                     : SelectionKind::Robot,
                select.substr(colon + 1) };
        }
        // A picture of the run: play it until the time of the picture.
        app.playing = app.playing ||
                      ((!app.screenshot.empty() || !app.screenshot_ui.empty()) &&
                       app.screenshot_at > 0.0);
    }
    runMainLoop(window, app);
    return app.error.empty() ? 0 : 1;
}
