// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "Robotik/Plugin/PluginSession.hpp"
#include "Robotik/Runtime/Simulation.hpp"

#include "Compages/Core/Units.hpp"
#include "Compages/World/World.hpp"

#include <filesystem>
#include <iostream>
#include <memory>
#include <optional>
#include <string>

#define HEADLESS_DT_S 0.01
#define HEADLESS_TIMEOUT_S 150.0

static void usage()
{
    std::cerr << "usage: Robotik-Headless scenario.yml [--seed N]\n";
}

int main(int argc, char** argv)
{
    std::optional<std::filesystem::path> path;
    std::optional<std::uint64_t> seed;
    for (int i = 1; i < argc; ++i)
    {
        std::string const argument = argv[i];
        if (argument == "--seed" && i + 1 < argc)
        {
            seed = std::stoull(argv[++i]);
        }
        else if (!path)
        {
            path = argument;
        }
        else
        {
            usage();
            return 1;
        }
    }
    if (!path)
    {
        usage();
        return 1;
    }

    try
    {
        robotik::PluginSession session;
        session.setHeadless(true);
        session.scan();
        robotik::PluginPrepare const prepared = session.prepare(*path);
        if (prepared != robotik::PluginPrepare::Ready)
        {
            std::cerr << (session.error().empty() ? "scenario is not a demo package"
                                                  : session.error())
                      << '\n';
            return 1;
        }
        robotik::Scenario scenario = robotik::Scenario::load(*path);
        compages::world::World world;
        robotik::Simulation simulation(
            world, std::move(scenario), nullptr, session.mission());
        session.attach(&simulation);
        if (session.start() != ROBOTIK_PLUGIN_OK)
        {
            std::cerr << session.error() << '\n';
            return 1;
        }
        if (seed)
        {
            simulation.reset(robotik::Seed{ *seed });
        }

        Seconds const dt(HEADLESS_DT_S);
        Seconds const timeout(HEADLESS_TIMEOUT_S);
        while (!simulation.finished() && simulation.time() < timeout)
        {
            if (session.ownsClock())
            {
                session.afterStep(HEADLESS_DT_S);
            }
            else
            {
                simulation.step(dt);
                session.afterStep(HEADLESS_DT_S);
            }
        }

        robotik::SkillScheduler const& skills = simulation.skills();
        robotik::ResourceManager const& resources =
            simulation.robot().resources();
        for (robotik::SkillRun const& run : skills.trace())
        {
            std::cout << "  " << run.start << "  " << skills.name(run.skill)
                      << "  " << robotik::toString(run.state);
            if (run.reason != robotik::SkillReason::None)
            {
                std::cout << " (" << robotik::toString(run.reason);
                if (run.blocker != robotik::NO_RESOURCE)
                {
                    std::cout << ": " << resources.name(run.blocker);
                }
                std::cout << ')';
            }
            std::cout << "  (" << run.end - run.start << ")\n";
        }
        bool passed = true;
        for (auto const& check : simulation.checks())
        {
            std::cout << (check.passed ? "[PASS] " : "[FAIL] ") << check.text;
            if (!check.detail.empty())
            {
                std::cout << "  (" << check.detail << ')';
            }
            std::cout << '\n';
            passed = passed && check.passed;
        }
        std::cout << "seed=" << simulation.seed().value
                  << " time=" << simulation.time().value() << "s\n";
        return passed ? 0 : 2;
    }
    catch (std::exception const& error)
    {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
