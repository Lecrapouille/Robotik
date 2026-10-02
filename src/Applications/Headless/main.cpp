// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "Robotik/Runtime/RobotRuntime.hpp"
#include "Robotik/Runtime/Simulation.hpp"

#include "Compages/Core/Units.hpp"
#include "Compages/World/World.hpp"

#include <filesystem>

#include <iostream>
#include <string>

#define HEADLESS_DT_S 0.01
#define HEADLESS_TIMEOUT_S 120.0

static char const* label(robotik::Status p_status)
{
    switch (p_status)
    {
        case robotik::Status::SUCCESS:
            return "success";
        case robotik::Status::FAILURE:
            return "failure";
        default:
            return "running";
    }
}

int main(int argc, char** argv)
{
    if (argc != 2)
    {
        std::cerr << "usage: Robotik-Headless scenario.yml\n";
        return 1;
    }

    try
    {
        compages::world::World world;
        robotik::Simulation simulation(
            world,
            nullptr,
            robotik::Scenario::load(std::filesystem::path(argv[1])));

        Seconds const dt(HEADLESS_DT_S);
        Seconds const timeout(HEADLESS_TIMEOUT_S);
        while (!simulation.finished() && simulation.runtime().time() < timeout)
        {
            simulation.step(dt);
        }

        for (auto const& entry : simulation.trace().entries)
        {
            std::cout << "  " << entry.start << "s  " << entry.name << "  "
                      << label(entry.status) << "  (" << entry.end - entry.start
                      << "s)\n";
        }
        bool passed = true;
        for (auto const& check : simulation.checks())
        {
            std::cout << (check.passed ? "[PASS] " : "[FAIL] ") << check.text
                      << '\n';
            passed = passed && check.passed;
        }
        std::cout << "time=" << simulation.runtime().time().value() << "s\n";
        return passed ? 0 : 2;
    }
    catch (std::exception const& error)
    {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
