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
    // Robotik-Simulator [scenario] [--screenshot file.png [--at seconds]]
    for (int i = 1; i < argc; ++i)
    {
        std::string const argument = argv[i];
        if (argument == "--screenshot" && i + 1 < argc)
        {
            app.screenshot = argv[++i];
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
    if (!app.scenario_path.empty())
    {
        app.load();
        // A picture of the run: play it until the time of the picture.
        app.playing = app.playing || !app.screenshot.empty();
    }
    runMainLoop(window, app);
    return app.error.empty() ? 0 : 1;
}
