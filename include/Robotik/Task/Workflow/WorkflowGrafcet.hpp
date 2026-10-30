// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file WorkflowGrafcet.hpp
/// The GRAFCET driving a cell, made from its workflow instead of drawn by
/// hand.
///
/// The workflow says who works (operators), what is made (products, one part
/// per lot), the route of each product, the skills of each operator to take
/// and give a part (recipes), and where parts wait between two operators
/// (buffers). The GRAFCET is the event graph of this flowshop (see
/// doc/Glossary.md): each operator works on the parts one after the other,
/// lot after lot, in the order of the products.
///
/// @code
/// for one part on one operator:
///   [idle . part given by the previous operator . place of the next buffer free]
///   (waits for the zone of the buffer it takes from)  take recipe  machining d s/X
///   (waits for the zone of the buffer it gives to)    give recipe
///   -> idle, "part done by <operator>", zone free
/// all the operators done -> End
/// @endcode
///
/// A step of the recipe is a skill call; its receptivity is @c done(skill)
/// and what follows " . " in the recipe. Machining is a time receptivity
/// (@c 2s/X10) while the operator holds the part. In a zone the operator
/// taking parts goes first: the other one waits.
///
/// With @c gemma: @c default the graphs Safety and Control of the default
/// GEMMA of TimedPetriNetEditor come first: a part starts only while Control
/// is in F1 (normal production), so @c Stop lets the operators finish the
/// parts they hold, and Control leaves A2 (end of cycle) for A1 when every
/// operator is between two parts.
///
/// @par Contract
/// - Promises: a GRAFCET graph named @c Production (after Safety and Control
///   with a GEMMA); the receptivity ending it (@c Production:X<n> of its
///   @c End step).
/// - Expects: a workflow read by @c loadWorkflow.
/// - Errors: no part to make, an operator of capacity above 1 (a GRAFCET
///   step holds one token: declare two operators), a product going from an
///   operator to the next one without a buffer between them, a buffer
///   without a place for the product, a @c {to} in the last recipe without
///   @c output.
#pragma once

#include "Robotik/Common/Result.hpp"
#include "Robotik/Task/WorkflowDescription.hpp"

#include <map>
#include <string>
#include <vector>

namespace tpne
{
class Net;
} // namespace tpne

namespace robotik
{

// ----------------------------------------------------------------------------
/// Generates the production GRAFCET of a workflow.
/// @param p_workflow Operators, recipes, buffers, products and their parts.
/// @param p_graphs Receives the graphs (replaced).
/// @return The receptivity true at the end of the production (@c Production:X126), or
///   what is missing in the workflow.
// ----------------------------------------------------------------------------
[[nodiscard]] Result<std::string> workflowGrafcet(WorkflowDescription const& p_workflow,
                                                  std::vector<tpne::Net>& p_graphs);

// ----------------------------------------------------------------------------
/// How many operations started in a run of a GRAFCET generated from this
/// workflow, read from its active steps: the steps of each operator follow
/// one another, so the operations before its active step have started.
/// @param p_workflow Workflow the GRAFCET was generated from.
/// @param p_graphs Graphs of the GRAFCET controller, now.
/// @return For each operation that started, by transition of the event
///   graph of @c analyseWorkflow (@c M1_P1: operator, product), the count.
// ----------------------------------------------------------------------------
[[nodiscard]] std::map<std::string, std::size_t>
startedOperations(WorkflowDescription const& p_workflow, std::vector<tpne::Net> const& p_graphs);

} // namespace robotik
