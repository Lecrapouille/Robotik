// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file RunReport.hpp
/// What a finished run leaves behind: outcome, verdicts and metrics, with
/// what is needed to replay it (scenario, case, seed).
#pragma once

#include "Robotik/Package/Scenario/Assertions.hpp"

#include <cstddef>
#include <cstdint>
#include <iosfwd>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace robotik
{

class Simulation;

// ****************************************************************************
/// One plan of the PDDL planner in a run: when, why, what came out.
///
/// @code
/// for (robotik::PlanSummary const& plan : record.planner.plans)
///     std::cout << plan.time << " s (" << plan.cause << "): "
///               << plan.result << ", " << plan.steps.size() << " steps\n";
/// @endcode
// ****************************************************************************
struct PlanSummary
{
    /// Simulated time of the search (s).
    double time = 0.0;
    /// "start", or what went wrong with the previous plan.
    std::string cause;
    /// Solved, Unsolvable, NoPlanFound or Invalid.
    std::string result;
    /// Ground actions, e.g. "pick(red_cube, table)".
    std::vector<std::string> steps;
    /// Number of states the search expanded.
    std::size_t expanded = 0;
    /// Time spent planning (wall clock).
    double milliseconds = 0.0;
};

// ****************************************************************************
/// What the planner did in a run. Empty without a planner.
///
/// @code
/// if (record.planner.used && record.planner.replans > 0)
///     std::cout << "replanned: " << record.planner.failure << '\n';
/// @endcode
// ****************************************************************************
struct PlannerSummary
{
    /// True when the scenario plans.
    bool used = false;
    /// Final state: Succeeded, Failed...
    std::string state;
    /// Number of plans after the first one.
    unsigned replans = 0;
    /// Why it failed, or why it last replanned.
    std::string failure;
    /// Every plan, in order.
    std::vector<PlanSummary> plans;
};

// ****************************************************************************
/// One run of one case with one seed.
///
/// @code
/// robotik::RunRecord const record =
///     robotik::RunRecord::capture(simulation, "nominal", { "time", "laps" });
/// std::cout << record.case_id << ": " << (record.passed ? "pass" : "FAIL") <<
/// '\n';
/// @endcode
// ****************************************************************************
struct RunRecord
{
    /// Case id in the experiment; empty for a single scenario.
    std::string case_id;
    /// Scenario id.
    std::string scenario;
    /// Seed of the run: replay with it.
    std::uint64_t seed = 0;
    /// How the run ended (e.g. "controller succeeded", "timeout").
    std::string outcome;
    /// Simulated time at the end (s).
    double time = 0.0;
    /// True when every assertion holds.
    bool passed = false;
    /// The ground truth stood in for perception (see @ref OraclePolicy).
    bool oracle = false;
    /// Physics fidelity, e.g. "abstract" or "abstract, red_cube: rigid":
    /// never compare metrics of runs at different levels.
    std::string physics;
    /// Verdict of each assertion.
    std::vector<Check> checks;
    /// Name and value; nothing when it could not be measured.
    std::vector<std::pair<std::string, std::optional<double>>> metrics;
    /// Plans and replans of the PDDL planner, if the scenario plans.
    PlannerSummary planner;
    /// Why the run could not be built or started, if it was not.
    std::string error;

    // -------------------------------------------------------------------------
    /// Takes a snapshot of a concluded simulation.
    /// @param p_simulation Simulation whose run has ended.
    /// @param p_case Case id to record.
    /// @param p_metrics Names or queries of the values to measure now.
    /// @return The record.
    // -------------------------------------------------------------------------
    [[nodiscard]] static RunRecord
    capture(Simulation const& p_simulation,
            std::string p_case,
            std::vector<std::string> const& p_metrics);
};

// ****************************************************************************
/// Every run of an experiment (or the single run of a scenario).
///
/// @code
/// robotik::RunReport report;
/// report.experiment = experiment.id;
/// report.runs.push_back(record);
/// report.writeText(std::cout);
/// std::ofstream json("report.json");
/// report.writeJson(json);
/// return report.passed() ? 0 : 2;
/// @endcode
// ****************************************************************************
struct RunReport
{
    /// Experiment id; empty for a single scenario.
    std::string experiment;
    /// Every run, in the order they ran.
    std::vector<RunRecord> runs;

    // -------------------------------------------------------------------------
    /// @return True when every run passed.
    // -------------------------------------------------------------------------
    [[nodiscard]] bool passed() const;

    // -------------------------------------------------------------------------
    /// Writes a summary for a human: one line per run, then the failed
    /// assertions.
    /// @param p_out Stream receiving the text.
    // -------------------------------------------------------------------------
    void writeText(std::ostream& p_out) const;

    // -------------------------------------------------------------------------
    /// Writes the whole report as JSON (for scripts comparing runs).
    /// @param p_out Stream receiving the JSON.
    // -------------------------------------------------------------------------
    void writeJson(std::ostream& p_out) const;
};

} // namespace robotik
