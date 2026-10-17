// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

//! @file PlanningEngine.hpp
//! @brief A PDDL planner behind a RobotIK interface (Kairos by default).
#pragma once

#include "Robotik/Planning/Pddl.hpp"

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace robotik
{

// ****************************************************************************
//! @brief How the planner searches. A run is reproducible: one thread, a
//! fixed search and no anytime improvement (they depend on the wall clock).
// ****************************************************************************
struct PlanningOptions
{
    //!< Search preset of the planner (Kairos: ff, lama, eager-lama...).
    std::string search = "lama";
    std::uint64_t seed = 0u;
    //!< Safety net only: a plan found before it does not depend on it.
    std::chrono::milliseconds timeout{ std::chrono::seconds(10) };
};

// ****************************************************************************
//! @brief Answer of the planner.
// ****************************************************************************
struct PlanningResult
{
    enum class Status : std::uint8_t
    {
        Solved,      //!< @ref steps reach the goal (empty: it already holds).
        Unsolvable,  //!< Proved: no plan exists from this state.
        NoPlanFound, //!< Search ended without a plan (time, memory).
        Invalid,     //!< Domain or problem rejected: see @ref diagnostics.
    };

    Status status = Status::Invalid;
    std::vector<GroundAction> steps;
    //!< Errors and warnings of the planner, one per line, with locations.
    std::string diagnostics;
    //!< Raw answer of the planner (status, plan text, statistics).
    std::string report;
    std::size_t expanded = 0u;
    double milliseconds = 0.0;

    [[nodiscard]] bool solved() const
    {
        return status == Status::Solved;
    }
};

[[nodiscard]] char const* toString(PlanningResult::Status p_status);

// ****************************************************************************
//! @brief Solves a problem written in PDDL.
// ****************************************************************************
class PlanningEngine
{
public:

    virtual ~PlanningEngine() = default;

    // -------------------------------------------------------------------------
    //! @brief Checks a domain. @return Empty when valid, the diagnostics
    //! otherwise. The domain is kept for the next @ref solve calls.
    // -------------------------------------------------------------------------
    [[nodiscard]] virtual std::string load(std::string const& p_domain,
                                           std::string const& p_file) = 0;

    // -------------------------------------------------------------------------
    //! @brief Plans for @p_problem against the loaded domain.
    // -------------------------------------------------------------------------
    [[nodiscard]] virtual PlanningResult
    solve(std::string const& p_problem, PlanningOptions const& p_options) = 0;
};

//! @brief The default engine: Kairos.
[[nodiscard]] std::unique_ptr<PlanningEngine> makeKairosPlanner();

} // namespace robotik
