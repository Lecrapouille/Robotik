// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file GrafcetDeadlocks.hpp
/// Finds, before the run, the situations where a GRAFCET and the resources
/// of its skills block each other (a deadlock).
///
/// A skill takes all its resources at once, so two skills alone never block
/// each other. A GRAFCET can: a step whose action holds a resource (a belt
/// that runs, an arm) until the step ends, while the step can only end after
/// another step whose skill needs that resource. The check builds one Petri
/// net of the steps and of the resources, and explores every marking it can
/// reach (TimedPetriNetEditor):
///
/// @code
/// step X (action S, S needs the arm)    resource place "arm" (1 token)
///   X --[X takes arm]--> X holds arm --[transition after X]--> next step, arm back
/// @endcode
///
/// Receptivities are taken as true some day (only the tokens decide), so a
/// marking without any transition to fire is a deadlock, unless the steps
/// of the normal end are active: the End step of a generated GRAFCET, else
/// the steps of the success of the scenario, else only last steps. The graphs
/// Safety and Control of a GEMMA are left out: they loop and never stop.
///
/// @par Contract
/// - Promises: the net analysed, the number of markings explored (all of
///   them, or the first @c p_max_states), each deadlock in words.
/// - Expects: graphs of a loaded GRAFCET controller, skills of its scheduler.
#pragma once

#include <cstddef>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace tpne
{
class Net;
} // namespace tpne

namespace robotik
{

class GrafcetController;
class SkillScheduler;

// ****************************************************************************
/// Result of @ref findDeadlocks.
// ****************************************************************************
struct GrafcetDeadlocks
{
    /// The net analysed: the steps of the graphs and one place per resource.
    std::shared_ptr<tpne::Net const> net;
    /// Markings explored.
    std::size_t states = 0U;
    /// False when the exploration stopped at the maximum of markings: the
    /// deadlocks found are only the ones of the explored part.
    bool complete = true;
    /// Each deadlock: the active steps and what they hold or wait for.
    std::vector<std::string> deadlocks;
};

// ----------------------------------------------------------------------------
/// Looks for deadlocks in graphs whose actions reserve resources.
/// @param p_graphs GRAFCET graphs, in their initial situation.
/// @param p_resources Exclusive resources of each action (a skill name).
/// @param p_success Receptivity of the normal end (its steps), or empty: a
///   marking where every active step is a last step is then the end.
/// @param p_max_states Markings explored at most.
/// @return What was found.
// ----------------------------------------------------------------------------
[[nodiscard]] GrafcetDeadlocks
findDeadlocks(std::vector<tpne::Net> const& p_graphs,
              std::map<std::string, std::vector<std::string>> const& p_resources,
              std::string const& p_success, std::size_t p_max_states = 100000U);

// ----------------------------------------------------------------------------
/// Looks for deadlocks in the GRAFCET of a cell (its initial situation) and
/// the exclusive resources of the skills of its actions.
/// @param p_grafcet The GRAFCET controller.
/// @param p_skills The scheduler of its skills.
/// @param p_max_states Markings explored at most.
/// @return What was found.
// ----------------------------------------------------------------------------
[[nodiscard]] GrafcetDeadlocks findDeadlocks(GrafcetController const& p_grafcet,
                                             SkillScheduler const& p_skills,
                                             std::size_t p_max_states = 100000U);

} // namespace robotik
