// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file Backchaining.hpp
/// The tree of PA-BT (Planning and Acting with Behavior Trees, Colledanchise,
/// Marzinotto and Ögren, 2019), as plain data: grown backwards from the goal
/// instead of searched forwards.
///
/// The tree starts with the conditions of the goal. When it fails on a
/// condition, the condition is expanded: the actions of the PDDL domain whose
/// effects make it true are put under it, each with its preconditions as new
/// conditions. Run as a behavior tree, a condition is
///
/// @code
/// ReactiveSelector                  in(red_cube, box)?   yes: done
///   in(red_cube, box)?              else run place(red_cube, box), which
///   ReactiveSequence                needs holding(red_cube)?   else run
///     holding(red_cube)?  <- expanded the same way when it fails
///     place(red_cube, box)
/// @endcode
///
/// and every tick reads the conditions on the beliefs again: an object that
/// falls from the gripper makes the tree pick it again, with no new search.
/// @ref PlanExecutor runs this tree when the scenario says
/// @c planner.method: @c pabt.
///
/// @par Contract
/// - Promises: the functions are pure; names are PDDL names (lower case).
///   An action is never added under a condition it would need itself
///   (no cycle).
/// - Expects: literals and actions of a domain read by @ref parseDomain.
/// - Errors: none; an action of an unknown name has no precondition.
#pragma once

#include "Robotik/Task/Planning/Pddl.hpp"

#include <cstddef>
#include <limits>
#include <map>
#include <set>
#include <string>
#include <vector>

namespace robotik
{

/// No action: a condition of the goal has no parent action.
inline constexpr std::size_t NO_ACTION =
    std::numeric_limits<std::size_t>::max();

// ****************************************************************************
/// A condition of the tree: a ground literal, read on the beliefs at each
/// tick.
// ****************************************************************************
struct BackchainCondition
{
    /// Ground literal (PDDL names).
    Literal literal;
    /// Action this condition is a precondition of; @ref NO_ACTION for a
    /// condition of the goal.
    std::size_t action = NO_ACTION;
    /// The actions that make it true were looked for (maybe none).
    bool expanded = false;
    /// Actions that make it true, in the order they are tried.
    std::vector<std::size_t> achievers;
};

// ****************************************************************************
/// An action of the tree: a ground action of the domain and its
/// preconditions.
// ****************************************************************************
struct BackchainAction
{
    /// Ground action (PDDL names).
    GroundAction action;
    /// Condition this action makes true.
    std::size_t condition = 0U;
    /// Its preconditions, as conditions of the tree.
    std::vector<std::size_t> preconditions;
};

// ****************************************************************************
/// The whole tree, in two flat lists: conditions and actions point at each
/// other by index.
///
/// @code
/// robotik::BackchainTree tree = robotik::backchainTree(spec.goal);
/// for (robotik::GroundAction const& a : robotik::achievers(schema, objects, goal))
///     robotik::attach(tree, 0U, a, robotik::groundPreconditions(schema, a));
/// @endcode
// ****************************************************************************
struct BackchainTree
{
    /// Every condition; the first ones are the goal.
    std::vector<BackchainCondition> conditions;
    /// Every action.
    std::vector<BackchainAction> actions;
    /// Conditions of the goal, in order.
    std::vector<std::size_t> goal;
};

// ----------------------------------------------------------------------------
/// @param p_goal Conjunction to reach (PDDL names).
/// @return A tree holding only the conditions of the goal.
// ----------------------------------------------------------------------------
[[nodiscard]] BackchainTree backchainTree(std::vector<Literal> const& p_goal);

// ----------------------------------------------------------------------------
/// Literals of a condition and of every condition above it: an action that
/// needs one of them would go round in a circle.
/// @param p_tree The tree.
/// @param p_condition A condition of the tree.
/// @return Its literal, then those above it up to the goal.
// ----------------------------------------------------------------------------
[[nodiscard]] std::vector<Literal> ancestors(BackchainTree const& p_tree,
                                             std::size_t p_condition);

// ----------------------------------------------------------------------------
/// Ground actions whose plain effects make @p_literal true. A parameter the
/// effect does not fix takes every object of its type.
///
/// @code
/// // pick(?o ?from) makes holding(?o) true: holding(red_cube) gives
/// // pick(red_cube, table) and pick(red_cube, shelf).
/// @endcode
///
/// @param p_schema The domain.
/// @param p_objects Every object and constant, with its type.
/// @param p_literal Ground literal to make true.
/// @param p_limit Largest number of actions returned.
/// @return The actions, in the order of the domain then of the objects.
// ----------------------------------------------------------------------------
[[nodiscard]] std::vector<GroundAction>
achievers(DomainSchema const& p_schema,
          std::map<std::string, std::string, std::less<>> const& p_objects,
          Literal const& p_literal,
          std::size_t p_limit = 256U);

// ----------------------------------------------------------------------------
/// @param p_schema The domain.
/// @param p_action A ground action of the domain.
/// @return Its plain preconditions, with its arguments in place of its
///   parameters.
// ----------------------------------------------------------------------------
[[nodiscard]] std::vector<Literal>
groundPreconditions(DomainSchema const& p_schema, GroundAction const& p_action);

// ----------------------------------------------------------------------------
/// @param p_schema The domain.
/// @return Predicates no action changes: a false one cannot become true, an
///   action that needs it is useless.
// ----------------------------------------------------------------------------
[[nodiscard]] std::set<std::string, std::less<>>
staticPredicates(DomainSchema const& p_schema);

// ----------------------------------------------------------------------------
/// Puts an action under a condition, with its preconditions as new
/// conditions (not expanded).
/// @param p_tree The tree.
/// @param p_condition Condition the action makes true.
/// @param p_action Ground action.
/// @param p_preconditions Its ground preconditions.
/// @return Index of the new action.
// ----------------------------------------------------------------------------
std::size_t attach(BackchainTree& p_tree,
                   std::size_t p_condition,
                   GroundAction p_action,
                   std::vector<Literal> const& p_preconditions);

} // namespace robotik
