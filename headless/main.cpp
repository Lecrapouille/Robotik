// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file main.cpp
/// Runs a scenario, or every case of an experiment, without a window. Given a
/// workflow (x.workflow.yaml), prints its cycle time and bottleneck instead.
///
/// Exit code: 0 when every run passed its assertions, 2 when one failed, 1
/// when a file or a plugin could not be loaded.

#include "Robotik/Package/Plugin/PluginSession.hpp"
#include "Robotik/Package/RunReport.hpp"
#include "Robotik/Package/Simulation.hpp"
#include "Robotik/Package/Scenario/Experiment.hpp"
#include "Robotik/Package/Scenario/FileRole.hpp"
#include "Robotik/Package/Scenario/Workflow.hpp"
#include "Robotik/Task/Workflow/WorkflowAnalysis.hpp"

#include "Compages/Core/Units.hpp"
#include "Compages/World/World.hpp"

#include <cstdio>
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
    /// Cycles of the schedule printed for a workflow.
    std::size_t cycles = 2U;
};

/// Prints the command line on the error stream.
static void usage()
{
    std::cerr << "usage: Robotik-Headless <x.scenario.yaml | x.rl.yaml | "
                 "x.experiment.yaml> [--seed N]\n"
                 "                        [--report report.json] [--trace]\n"
                 "       Robotik-Headless <x.workflow.yaml> [--cycles N]\n";
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
        else if (argument == "--cycles" && i + 1 < argc)
        {
            options.cycles = std::stoul(argv[++i]);
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
    robotik::SkillScheduler const& skills = p_simulation.cell().skills();
    robotik::ResourceManager const& resources =
        p_simulation.cell().robot().resources();
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
    robotik::Seconds const dt(HEADLESS_DT_S);
    double const timeout = simulation.scenario().timeout.value();
    double const limit = timeout > 0.0 ? timeout : HEADLESS_TIMEOUT_S;
    double simulated = 0.0;
    double host = 0.0;
    robotik::Seconds last = simulation.time();
    while (!simulation.finished() && simulated < limit &&
           host < HEADLESS_TIMEOUT_S)
    {
        if (!p_session.ownsClock())
        {
            simulation.step(dt);
        }
        p_session.afterStep(HEADLESS_DT_S);
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
    robotik::RunRecord record =
        robotik::captureRun(simulation, p_case, p_metrics);
    p_session.stop();
    p_session.detach();
    return record;
}

/// Prints the cycle time, the throughput, the critical circuit and the
/// first cycles of a workflow.
/// @return Exit code: 0, or 1 when the file is wrong.
static int analyse(Options const& p_options)
{
    robotik::Result<robotik::WorkflowDescription> workflow =
        robotik::loadWorkflow(p_options.file);
    if (!workflow)
    {
        std::cerr << workflow.error() << '\n';
        return 1;
    }
    robotik::Result<robotik::WorkflowAnalysis> result =
        robotik::analyseWorkflow(workflow.value(), p_options.cycles);
    if (!result)
    {
        std::cerr << result.error() << '\n';
        return 1;
    }
    robotik::WorkflowAnalysis const& analysis = result.value();
    std::cout << "workflow " << workflow.value().id << ": "
              << workflow.value().operators.size() << " operators, "
              << workflow.value().products.size() << " products\n"
              << "cycle time: " << analysis.cycle_time.value() << " s\n"
              << "throughput: " << analysis.throughput * 60.0
              << " products/min\n"
              << "critical circuit:\n";
    for (std::string const& operation : analysis.critical)
    {
        std::cout << "  " << operation << '\n';
    }
    std::cout << "earliest schedule (" << p_options.cycles << " cycles):\n";
    for (robotik::WorkflowBar const& bar : analysis.bars)
    {
        std::printf("  %7.2f s .. %7.2f s  %-10s %-10s %s #%zu\n",
                    bar.start.value(), bar.end.value(), bar.on.c_str(),
                    bar.product.c_str(), bar.operation.c_str(), bar.cycle + 1U);
    }
    return 0;
}

int main(int argc, char** argv)
{
    std::optional<Options> const options = parse(argc, argv);
    if (!options)
    {
        usage();
        return 1;
    }
    if (robotik::fileRole(options->file) == robotik::FileRole::Workflow)
    {
        return analyse(*options);
    }

    try
    {
        robotik::PluginSession session;
        session.setHeadless(true);
        session.scan();
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
                    robotik::Result<robotik::Scenario> scenario =
                        robotik::scenarioOf(experiment, item, session.assets());
                    if (!scenario)
                    {
                        std::cerr << item.id << ": " << scenario.error() << '\n';
                        return 1;
                    }
                    report.runs.push_back(run(session,
                                              item.scenario,
                                              scenario.take(),
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
            robotik::Result<robotik::Scenario> loaded =
                robotik::loadScenario(options->file, session.assets());
            if (!loaded)
            {
                std::cerr << loaded.error() << '\n';
                return 1;
            }
            robotik::Scenario scenario = loaded.take();
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
