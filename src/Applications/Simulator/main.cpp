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
    if (argc > 1)
    {
        app.scenario_path = std::filesystem::path(argv[1]);
        if (app.scenario_path.filename() == "fly_obstacle_avoidance.yml")
        {
            app.kind = HostedMission::Fly;
        }
    }
    app.load();

    runMainLoop(window, app);
    return 0;
}
