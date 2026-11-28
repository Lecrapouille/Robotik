// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file main.cpp
/// Runs a scenario, or every case of an experiment, without a window.
///
/// Exit code: 0 when every run passed its assertions, 2 when one failed, 1
/// when a file or a plugin could not be loaded.

#include "RobotIK/Package/Plugin/PluginSession.hpp"
#include "RobotIK/Package/RunReport.hpp"
#include "RobotIK/Package/StandaloneSimulation.hpp"
#include "RobotIK/Task/Story/Story.hpp"
#include "RobotIK/Package/Scenario/Experiment.hpp"

#include "Compages/Core/Units.hpp"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>
#include <string>

#define HEADLESS_DT_S 0.01
/// Safety net for scenarios without a timeout of their own.
#define HEADLESS_TIMEOUT_S 600.0

/// Command line of RobotIK-Headless.
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
    std::cerr << "usage: RobotIK-Headless <x.scenario.yaml | x.rl.yaml | "
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
    if (robotik::Story const* story = p_simulation.cell().story())
    {
        for (std::string const& line : story->log())
        {
            std::cout << "  story  " << line << '\n';
        }
    }
    robotik::SkillScheduler const& skills = p_simulation.cell().skills();
    robotik::ResourceManager const& resources =
        p_simulation.cell().robot().getResources();
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
            else if (!run.failure.code.empty())
            {
                std::cout << ": " << run.failure.code;
            }
            std::cout << ')';
            if (!run.failure.why.empty())
            {
                std::cout << "  " << run.failure.why;
            }
        }
        std::cout << "  (" << run.end - run.start << ")\n";
    }
    // The plans of the PDDL planner: when, why, what.
    if (robotik::PlanExecutor const* planner = p_simulation.cell().planner())
    {
        std::size_t index = 0U;
        for (robotik::PlanningRecord const& record : planner->history())
        {
            std::cout << "  plan " << index++ << " at " << record.time << " ("
                      << record.cause
                      << "): " << robotik::toString(record.result.status);
            for (robotik::GroundAction const& step : record.result.steps)
            {
                std::cout << ' ' << robotik::toPddl(step);
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
/// One run of an opened simulation (its plugin, its world, its scenario): it
/// goes until the scenario says it ends. @p_opened failed: the record says
/// why.
// ----------------------------------------------------------------------------
static robotik::RunRecord
run(robotik::Result<robotik::StandaloneSimulation> p_opened,
    std::string const& p_scenario,
    std::string const& p_case,
    std::optional<std::uint64_t> p_seed,
    std::vector<std::string> const& p_metrics,
    bool p_trace)
{
    if (!p_opened)
    {
        robotik::RunRecord failed;
        failed.case_id = p_case;
        failed.scenario = p_scenario;
        failed.seed = p_seed.value_or(0U);
        failed.error = p_opened.error();
        return failed;
    }
    robotik::StandaloneSimulation& opened = p_opened.value();
    robotik::Simulation& simulation = opened.simulation();
    if (p_seed)
    {
        simulation.reset(robotik::Seed{ *p_seed });
    }

    // Simulated time is summed across resets: a plugin that owns the clock
    // may restart episodes and rewind it. The scenario timeout bounds the
    // whole run; the host time bounds a plugin that never advances it.
    robotik::Seconds const dt(HEADLESS_DT_S);
    double const timeout = simulation.scenario().timeout.value();
    double const limit = timeout > 0.0 ? timeout : HEADLESS_TIMEOUT_S;
    double simulated = 0.0;
    double host = 0.0;
    robotik::Seconds last = simulation.time();
    while (!simulation.finished() && simulated < limit &&
           host < HEADLESS_TIMEOUT_S)
    {
        opened.step(dt);
        robotik::Seconds const now = simulation.time();
        simulated += now >= last ? (now - last).value() : now.value();
        last = now;
        host += HEADLESS_DT_S;
    }
    simulation.conclude();
    if (p_trace)
    {
        printTrace(simulation);
    }
    return robotik::captureRun(simulation, p_case, p_metrics);
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
        robotik::RunReport report;

        if (robotik::isExperiment(options->file))
        {
            robotik::Result<robotik::Experiment> loaded =
                robotik::loadExperiment(options->file);
            if (!loaded)
            {
                std::cerr << loaded.error() << '\n';
                return 1;
            }
            robotik::Experiment const experiment = loaded.take();
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
                    // Each run loads its plugin and reads its files again: a
                    // case never inherits the world nor the plugin state of
                    // the previous one.
                    robotik::StandaloneSimulation::PluginSetup setup;
                    setup.scenario =
                        [&experiment, &item](robotik::AssetResolver const& p_assets)
                    { return robotik::scenarioOf(experiment, item, p_assets); };
                    report.runs.push_back(
                        run(robotik::StandaloneSimulation::openPlugin(
                                item.scenario, std::move(setup)),
                            item.scenario.stem().string(),
                            item.id,
                            seed,
                            experiment.metrics,
                            options->trace));
                    robotik::RunRecord const& last = report.runs.back();
                    if (!last.error.empty())
                    {
                        std::cerr << item.id << ": " << last.error << '\n';
                    }
                    std::cout << (last.passed ? "[PASS] " : "[FAIL] ")
                              << item.id << " seed=" << last.seed << std::endl;
                }
            }
        }
        else
        {
            auto opened = robotik::StandaloneSimulation::openPlugin(options->file);
            if (!opened)
            {
                std::cerr << opened.error() << '\n';
                return 1;
            }
            report.experiment = opened.value().simulation().scenario().id;
            report.runs.push_back(run(std::move(opened),
                                      options->file.stem().string(),
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

        robotik::writeText(report, std::cout);
        if (!options->report.empty())
        {
            std::ofstream file(options->report);
            robotik::writeJson(report, file);
            std::cout << "report: " << options->report.string() << '\n';
        }
        for (robotik::RunRecord const& record : report.runs)
        {
            if (!record.error.empty())
            {
                return 1;
            }
        }
        return robotik::passed(report) ? 0 : 2;
    }
    catch (std::exception const& error)
    {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
