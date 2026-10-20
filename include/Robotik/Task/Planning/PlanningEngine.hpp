// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file PlanningEngine.hpp
/// A PDDL planner behind a RobotIK interface (Kairos by default).
#pragma once

#include "Robotik/Task/Planning/Pddl.hpp"

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace robotik
{

// ****************************************************************************
/// How the planner searches. A run is reproducible: one thread, a fixed
/// search and no anytime improvement (they depend on the wall clock).
///
/// @code
/// robotik::PlanningOptions options;
/// options.search = "ff";
/// options.timeout = std::chrono::seconds(2);
/// @endcode
// ****************************************************************************
struct PlanningOptions
{
    /// Search preset of the planner (Kairos: ff, lama, eager-lama...).
    std::string search = "lama";
    /// Seed of the search, for the planners that draw at random.
    std::uint64_t seed = 0u;
    /// Safety net only: a plan found before it does not depend on it.
    std::chrono::milliseconds timeout{ std::chrono::seconds(10) };
};

// ****************************************************************************
/// Answer of the planner.
///
/// @code
/// robotik::PlanningResult const result = planner->solve(problem, options);
/// if (result.solved())
///     for (auto const& step : result.steps)
///         std::cout << step.name << '\n';
/// else
///     std::cerr << robotik::toString(result.status) << '\n' <<
///     result.diagnostics;
/// @endcode
// ****************************************************************************
struct PlanningResult
{
    // ------------------------------------------------------------------------
    /// How the search ended.
    // ------------------------------------------------------------------------
    enum class Status : std::uint8_t
    {
        Solved,      ///< @ref steps reach the goal (empty: it already holds).
        Unsolvable,  ///< Proved: no plan exists from this state.
        NoPlanFound, ///< Search ended without a plan (time, memory).
        Invalid,     ///< Domain or problem rejected: see @ref diagnostics.
    };

    /// How the search ended.
    Status status = Status::Invalid;
    /// The plan, in order, when @ref status is @c Solved.
    std::vector<GroundAction> steps;
    /// Errors and warnings of the planner, one per line, with locations.
    std::string diagnostics;
    /// Raw answer of the planner (status, plan text, statistics).
    std::string report;
    /// Number of states the search expanded.
    std::size_t expanded = 0u;
    /// Time spent planning (wall clock).
    double milliseconds = 0.0;
};

// ------------------------------------------------------------------------
/// @return True when a plan was found.
// ------------------------------------------------------------------------
[[nodiscard]] inline bool solved(PlanningResult const& p_result)
{
    return p_result.status == PlanningResult::Status::Solved;
}

// ----------------------------------------------------------------------------
/// Names a planning status, for logs.
/// @param p_status Status to name.
/// @return For example "solved" or "unsolvable".
// ----------------------------------------------------------------------------
[[nodiscard]] char const* toString(PlanningResult::Status p_status);

// ****************************************************************************
/// Solves a problem written in PDDL.
///
/// Load the domain once, then call @ref solve for each new state of the
/// world (each replan).
///
/// @code
/// auto planner = robotik::makeKairosPlanner();
/// std::string const errors = planner->load(domain_text, "pick.domain.pddl");
/// if (!errors.empty())
///     throw std::runtime_error(errors);
/// robotik::PlanningResult const plan = planner->solve(problem_text, {});
/// @endcode
// ****************************************************************************
class PlanningEngine
{
public:

    virtual ~PlanningEngine() = default;

    // -------------------------------------------------------------------------
    /// Checks a domain. The domain is kept for the next @ref solve calls.
    /// @param p_domain PDDL text of the domain.
    /// @param p_file File name, used in the error locations.
    /// @return Empty when valid, the diagnostics otherwise.
    // -------------------------------------------------------------------------
    [[nodiscard]] virtual std::string load(std::string const& p_domain,
                                           std::string const& p_file) = 0;

    // -------------------------------------------------------------------------
    /// Plans against the loaded domain.
    /// @param p_problem PDDL text of the problem (objects, initial state,
    ///   goal).
    /// @param p_options Search preset, seed and timeout.
    /// @return The plan, or why there is none.
    // -------------------------------------------------------------------------
    [[nodiscard]] virtual PlanningResult
    solve(std::string const& p_problem, PlanningOptions const& p_options) = 0;
};

// ----------------------------------------------------------------------------
/// Creates the default engine.
/// @return A Kairos planner with no domain loaded.
// ----------------------------------------------------------------------------
[[nodiscard]] std::unique_ptr<PlanningEngine> makeKairosPlanner();

} // namespace robotik
