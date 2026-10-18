// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file PlanGraph.hpp
/// @brief Which steps of a plan must stay in order, and which may run at the
/// same time (partial order).
///
/// Each step has two events: its start (its skills begin; its precondition,
/// or @c at @c start / @c over @c all condition, must hold) and its end (its
/// skills are done; its effects hold). Events keep the order of the plan
/// when:
/// - one changes an atom the other reads, or both change it;
/// - their steps need the same resource and one of them alone (an arm): the
///   first one ends before the second one starts.
///
/// @code
/// sequential plan: 1. pick(left, a)  2. pick(right, b)  3. place(left, a, bin)
///   end 1 -> start 3 (3 needs holding(left, a)); 2 runs with 1 and 3.
/// temporal plan: carry(left, tray) [0, 10]   visit(right, glass) [1, 4]
///   start carry -> start visit (visit needs held(tray) over all)
///   end visit -> end carry (carry ends the holding): the tray is held all along.
/// @endcode
///
/// Any execution that keeps these pairs leads to the same final state as the
/// plan. A variable of a quantified formula (@c ?x) matches any argument.
#pragma once

#include "Robotik/Planning/Pddl.hpp"
#include "Robotik/Runtime/Resources.hpp"

#include <cstddef>
#include <span>
#include <vector>

namespace robotik
{

// ****************************************************************************
/// @brief What one ground step of a plan reads, changes and holds.
// ****************************************************************************
struct StepFootprint
{
    /// Ground atoms of each end (PDDL names).
    ActionSchema::Phase start;
    ActionSchema::Phase end;
    /// Resources of the skills the step runs.
    std::vector<ResourceRequirement> resources;
    /// Times of both ends in the plan.
    double start_time = 0.0;
    double end_time = 0.0;
};

// ****************************************************************************
/// @brief The start or the end of a step.
// ****************************************************************************
struct PlanEvent
{
    std::size_t step = 0u;
    bool end = false;

    bool operator==(PlanEvent const&) const = default;
};

// ****************************************************************************
/// @brief Events a step waits for: before it starts, before it ends (its
/// own start is implied).
// ****************************************************************************
struct StepOrder
{
    std::vector<PlanEvent> start_after;
    std::vector<PlanEvent> end_after;
};

// ----------------------------------------------------------------------------
/// @brief Footprint of @p_step for its action @p_action: its parameters
/// replaced by the arguments of the step, its times taken from the plan.
// ----------------------------------------------------------------------------
[[nodiscard]] StepFootprint footprint(ActionSchema const& p_action,
                                      GroundAction const& p_step);

// ----------------------------------------------------------------------------
/// @brief Direct predecessors of the events of each step (see the file
/// comment), without those implied by others (transitive reduction).
// ----------------------------------------------------------------------------
[[nodiscard]] std::vector<StepOrder> orderSteps(std::span<StepFootprint const> p_steps);

// ----------------------------------------------------------------------------
/// @brief True when each step starts once the previous one ended, and
/// waits for nothing else: the plan runs step by step.
// ----------------------------------------------------------------------------
[[nodiscard]] bool sequential(std::span<StepOrder const> p_order);

} // namespace robotik
