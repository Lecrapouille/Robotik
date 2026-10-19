// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file PlanGraph.hpp
/// Which steps of a plan must stay in order, and which may run at the same
/// time (partial order).
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
///   end visit -> end carry (carry ends the holding): the tray is held all
///   along.
/// @endcode
///
/// Any execution that keeps these pairs leads to the same final state as the
/// plan. A variable of a quantified formula (@c ?x) matches any argument.
#pragma once

#include "Robotik/Task/Planning/Pddl.hpp"
#include "Robotik/Robot/Resources.hpp"

#include <cstddef>
#include <span>
#include <vector>

namespace robotik
{

// ****************************************************************************
/// What one ground step of a plan reads, changes and holds.
///
/// Built by @ref footprint, then completed with the resources of the skills
/// the step runs.
///
/// @code
/// robotik::StepFootprint f = robotik::footprint(domain.actions.at("pick"),
/// step); f.resources = { resources.require("left_arm") };
/// @endcode
// ****************************************************************************
struct StepFootprint
{
    /// Ground atoms read and changed at the start (PDDL names).
    ActionSchema::Phase start;
    /// Ground atoms read and changed at the end (PDDL names).
    ActionSchema::Phase end;
    /// Resources of the skills the step runs.
    std::vector<ResourceRequirement> resources;
    /// Time of the start in the plan.
    double start_time = 0.0;
    /// Time of the end in the plan.
    double end_time = 0.0;
};

// ****************************************************************************
/// The start or the end of a step.
///
/// @code
/// robotik::PlanEvent const end_of_first{ 0u, true };
/// @endcode
// ****************************************************************************
struct PlanEvent
{
    /// Index of the step in the plan.
    std::size_t step = 0u;
    /// True for the end of the step, false for its start.
    bool end = false;

    /// Same step, same end.
    bool operator==(PlanEvent const&) const = default;
};

// ****************************************************************************
/// Events a step waits for: before it starts, before it ends (its own start
/// is implied).
///
/// @code
/// for (robotik::PlanEvent const& e : order[2].start_after)
///     std::cout << "step 2 starts after " << (e.end ? "end" : "start")
///               << " of step " << e.step << '\n';
/// @endcode
// ****************************************************************************
struct StepOrder
{
    /// Events that must happen before the step starts.
    std::vector<PlanEvent> start_after;
    /// Events that must happen before the step ends.
    std::vector<PlanEvent> end_after;
};

// ----------------------------------------------------------------------------
/// Computes what a step reads and changes: the parameters of its action
/// replaced by the arguments of the step, its times taken from the plan.
/// @param p_action Action schema of the domain.
/// @param p_step Step of the plan that uses this action.
/// @return The footprint, without resources.
// ----------------------------------------------------------------------------
[[nodiscard]] StepFootprint footprint(ActionSchema const& p_action,
                                      GroundAction const& p_step);

// ----------------------------------------------------------------------------
/// Orders the events of a plan (see the file comment). Only the direct
/// predecessors are kept, not those implied by others (transitive
/// reduction).
/// @param p_steps Footprints of the steps, in plan order.
/// @return One @ref StepOrder per step.
// ----------------------------------------------------------------------------
[[nodiscard]] std::vector<StepOrder>
orderSteps(std::span<StepFootprint const> p_steps);

// ----------------------------------------------------------------------------
/// Tells whether a plan runs step by step.
/// @param p_order Result of @ref orderSteps.
/// @return True when each step starts once the previous one ended, and
///   waits for nothing else.
// ----------------------------------------------------------------------------
[[nodiscard]] bool sequential(std::span<StepOrder const> p_order);

} // namespace robotik
