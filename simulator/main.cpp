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
    if (argc > 1)
    {
        app.scenario_path = std::filesystem::path(argv[1]);
    }
    else
    {
        for (robotik::PluginPackage const& package : app.plugins.catalog().packages())
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
    }
    runMainLoop(window, app);
    return app.error.empty() ? 0 : 1;
}
