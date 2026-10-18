// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file PlanGraph.hpp
/// @brief Which steps of a sequential plan must stay in order, and which
/// may run at the same time (partial order).
///
/// @code
/// plan: 1. pick(left, a)  2. pick(right, b)  3. place(left, a, bin)
/// 1 -> 3 (3 needs holding(left, a)), 2 alone: 1 and 2 run together.
/// @endcode
///
/// Step @c i (before @c j in the plan) stays before @c j when:
/// - @c i changes an atom @c j reads (causal link, or @c j would be undone);
/// - @c j changes an atom @c i reads (@c i needs it as it was);
/// - both change the same atom (the last one must win);
/// - their skills need the same resource and one of them alone (an arm).
///
/// Any order that keeps these pairs leads to the same final state as the
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
    /// Ground atoms of the precondition (PDDL names).
    std::vector<Atom> reads;
    /// Ground atoms the effects may change.
    std::vector<Atom> writes;
    /// Resources of the skills the step runs.
    std::vector<ResourceRequirement> resources;
};

// ----------------------------------------------------------------------------
/// @brief Footprint of @p_step for its action @p_action: its parameters
/// replaced by the arguments of the step.
// ----------------------------------------------------------------------------
[[nodiscard]] StepFootprint footprint(ActionSchema const& p_action,
                                      GroundAction const& p_step);

// ----------------------------------------------------------------------------
/// @brief Direct predecessors of each step: the pairs that must stay in
/// order (see the file comment), without those implied by others
/// (transitive reduction). A chain @c {}, @c {0}, @c {1}... means fully
/// sequential.
// ----------------------------------------------------------------------------
[[nodiscard]] std::vector<std::vector<std::size_t>>
orderSteps(std::span<StepFootprint const> p_steps);

/// @brief True when each step comes right after the previous one.
[[nodiscard]] bool sequential(std::vector<std::vector<std::size_t>> const& p_after);

} // namespace robotik
