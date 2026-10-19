// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file main.cpp
/// @brief Runs a scenario, or every case of an experiment, without a window.
///
/// Exit code: 0 when every run passed its assertions, 2 when one failed, 1
/// when a file or a plugin could not be loaded.

#include "Robotik/Plugin/PluginSession.hpp"
#include "Robotik/Runtime/RunReport.hpp"
#include "Robotik/Runtime/Simulation.hpp"
#include "Robotik/Scenario/Experiment.hpp"

#include "Compages/Core/Units.hpp"
#include "Compages/World/World.hpp"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>
#include <string>

#define HEADLESS_DT_S 0.01
/// Safety net for scenarios without a timeout of their own.
#define HEADLESS_TIMEOUT_S 600.0

/// Command line of Robotik-Headless.
struct Options
{
    std::filesystem::path file;
    std::optional<std::uint64_t> seed;
    std::filesystem::path report;
    bool trace = false;
};

/// Prints the command line on the error stream.
static void usage()
{
    std::cerr << "usage: Robotik-Headless <x.scenario.yaml | x.rl.yaml | "
                 "x.experiment.yaml> [--seed N]\n"
                 "                        [--report report.json] [--trace]\n";
}

/// Reads the scenario path and the optional flags.
static std::optional<Options> parse(int argc, char** argv)
{
    Options options;
    for (int i = 1; i < argc; ++i)
    {
        std::string const argument = argv[i];
        if (argument == "--seed" && i + 1 < argc)
        {
            options.seed = std::stoull(argv[++i]);
        }
        else if (argument == "--report" && i + 1 < argc)
        {
            options.report = argv[++i];
        }
        else if (argument == "--trace")
        {
            options.trace = true;
        }
        else if (options.file.empty())
        {
            options.file = argument;
        }
        else
        {
            return std::nullopt;
        }
    }
    if (options.file.empty())
    {
        return std::nullopt;
    }
    return options;
}

/// Prints each skill run of the finished simulation.
static void printTrace(robotik::Simulation const& p_simulation)
{
    robotik::SkillScheduler const& skills = p_simulation.skills();
    robotik::ResourceManager const& resources =
        p_simulation.robot().resources();
    for (robotik::SkillRun const& run : skills.trace())
    {
        std::cout << "  " << run.start << "  " << skills.name(run.skill) << "  "
                  << robotik::toString(run.state);
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
    // The plans of the PDDL planner: when, why, what.
    if (robotik::PlanExecutor const* planner = p_simulation.planner())
    {
        std::size_t index = 0u;
        for (robotik::PlanningRecord const& record : planner->history())
        {
            std::cout << "  plan " << index++ << " at " << record.time << " ("
                      << record.cause
                      << "): " << robotik::toString(record.result.status);
            for (robotik::GroundAction const& step : record.result.steps)
            {
                std::cout << ' ' << step.toPddl();
            }
            std::cout << '\n';
        }
        if (planner->state() == robotik::PlanState::Failed)
        {
            std::cout << "  planner: " << robotik::toString(planner->state())
                      << ", " << planner->failure() << '\n';
        }
    }
}

// ----------------------------------------------------------------------------
/// One run: the plugin owning @p_file brings the mission, the world is built
/// from @p_scenario, the run goes until the scenario says it ends.
// ----------------------------------------------------------------------------
static robotik::RunRecord run(robotik::PluginSession& p_session,
                              std::filesystem::path const& p_file,
                              robotik::Scenario p_scenario,
                              std::string const& p_case,
                              std::optional<std::uint64_t> p_seed,
                              std::vector<std::string> const& p_metrics,
                              bool p_trace)
{
    robotik::RunRecord failed;
    failed.case_id = p_case;
    failed.scenario = p_scenario.id;
    failed.seed = p_seed.value_or(p_scenario.seed);
    if (p_session.prepare(p_file) != robotik::PluginPrepare::Ready)
    {
        failed.error = p_session.error().empty()
                           ? "scenario is not in a plugin package"
                           : p_session.error();
        return failed;
    }
    compages::world::World world;
    robotik::Simulation simulation(
        world, std::move(p_scenario), nullptr, p_session.mission());
    p_session.attach(&simulation);
    if (p_session.start() != ROBOTIK_PLUGIN_OK)
    {
        failed.error = p_session.error();
        p_session.detach();
        return failed;
    }
    if (p_seed)
    {
        simulation.reset(robotik::Seed{ *p_seed });
    }

    // Simulated time is summed across resets: a plugin that owns the clock
    // may restart episodes and rewind it. The scenario timeout bounds the
    // whole run; the host time bounds a plugin that never advances it.
    Seconds const dt(HEADLESS_DT_S);
    double const timeout = simulation.scenario().timeout.value();
    double const limit = timeout > 0.0 ? timeout : HEADLESS_TIMEOUT_S;
    double simulated = 0.0;
    double host = 0.0;
    Seconds last = simulation.time();
    while (!simulation.finished() && simulated < limit &&
           host < HEADLESS_TIMEOUT_S)
    {
        if (!p_session.ownsClock())
        {
            simulation.step(dt);
        }
        p_session.afterStep(HEADLESS_DT_S);
        Seconds const now = simulation.time();
        simulated += now >= last ? (now - last).value() : now.value();
        last = now;
        host += HEADLESS_DT_S;
    }
    simulation.conclude();
    if (p_trace)
    {
        printTrace(simulation);
    }
    robotik::RunRecord record =
        robotik::RunRecord::capture(simulation, p_case, p_metrics);
    p_session.stop();
    p_session.detach();
    return record;
}

int main(int argc, char** argv)
{
    std::optional<Options> const options = parse(argc, argv);
    if (!options)
    {
        usage();
        return 1;
    }

    try
    {
        robotik::PluginSession session;
        session.setHeadless(true);
        session.scan();
        robotik::RunReport report;

        if (robotik::Experiment::describes(options->file))
        {
            robotik::Experiment const experiment =
                robotik::Experiment::load(options->file);
            report.experiment = experiment.id;
            std::vector<std::optional<std::uint64_t>> seeds;
            if (options->seed)
            {
                seeds.emplace_back(*options->seed);
            }
            for (std::uint64_t seed : experiment.seeds)
            {
                if (!options->seed)
                {
                    seeds.emplace_back(seed);
                }
            }
            if (seeds.empty())
            {
                seeds.emplace_back(std::nullopt);
            }
            for (robotik::ExperimentCase const& item : experiment.cases)
            {
                for (std::optional<std::uint64_t> const seed : seeds)
                {
                    // A scenario outside the catalog brings its package (and
                    // its asset pack) along.
                    if (session.catalog()
                            .matchScenario(item.scenario)
                            .package == nullptr)
                    {
                        (void)session.prepare(item.scenario);
                    }
                    // Each run reads its files again: a case never inherits the
                    // world of the previous one.
                    robotik::Scenario scenario =
                        experiment.scenario(item, session.assets());
                    report.runs.push_back(run(session,
                                              item.scenario,
                                              std::move(scenario),
                                              item.id,
                                              seed,
                                              experiment.metrics,
                                              options->trace));
                    robotik::RunRecord const& last = report.runs.back();
                    std::cout << (last.passed ? "[PASS] " : "[FAIL] ")
                              << item.id << " seed=" << last.seed << std::endl;
                }
            }
        }
        else
        {
            if (session.prepare(options->file) != robotik::PluginPrepare::Ready)
            {
                std::cerr << (session.error().empty()
                                  ? "scenario is not in a plugin package"
                                  : session.error())
                          << '\n';
                return 1;
            }
            robotik::Scenario scenario =
                robotik::Scenario::load(options->file, session.assets());
            report.experiment = scenario.id;
            report.runs.push_back(run(session,
                                      options->file,
                                      std::move(scenario),
                                      "default",
                                      options->seed,
                                      { "time" },
                                      true));
            for (robotik::Check const& check : report.runs.back().checks)
            {
                std::cout << '[' << robotik::toString(check.verdict) << "] "
                          << check.text << "  (" << check.detail << ")\n";
            }
        }

        session.release();
        report.writeText(std::cout);
        if (!options->report.empty())
        {
            std::ofstream file(options->report);
            report.writeJson(file);
            std::cout << "report: " << options->report.string() << '\n';
        }
        for (robotik::RunRecord const& record : report.runs)
        {
            if (!record.error.empty())
            {
                return 1;
            }
        }
        return report.passed() ? 0 : 2;
    }
    catch (std::exception const& error)
    {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
