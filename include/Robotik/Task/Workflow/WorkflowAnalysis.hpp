// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file WorkflowAnalysis.hpp
/// How fast a cell can work, computed from its workflow without running it.
///
/// The workflow becomes a timed event graph (a Petri net where each place
/// has one input and one output, see doc/Glossary.md): one transition per
/// operation, one circuit per operator (its capacity is the number of
/// tokens) and one per product kind (count tokens). In the (max,+) algebra
/// the cell repeats with a period, the @e cycle @e time λ: the longest
/// average circuit, the @e critical circuit, is the bottleneck. Shorten an
/// operation of the critical circuit and λ goes down; shorten another one and
/// nothing changes.
///
/// @code
/// operators: camera, arm            products: red_cube (count 1)
///   red_cube: detect on camera (0.5 s) -> move on arm (6 s)
/// -> circuit of red_cube: 0.5 + 6 = 6.5 s     circuit of arm: 6 s
/// -> λ = 6.5 s, throughput 1 product / 6.5 s, critical: detect, move
/// @endcode
///
/// @par Contract
/// - Promises: the cycle time, the critical operations and the earliest
///   schedule of the first cycles (Gantt bars), from the declared durations
///   or from the durations measured in a run.
/// - Expects: a workflow checked by @ref loadWorkflow.
/// - Errors: a workflow the (max,+) algorithms cannot solve is a failure.
#pragma once

#include "Robotik/Common/Result.hpp"
#include "Robotik/Task/Scheduler.hpp"
#include "Robotik/Task/WorkflowDescription.hpp"

#include <iosfwd>
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

// ****************************************************************************
/// One operation during one cycle, for a Gantt chart.
// ****************************************************************************
struct WorkflowBar
{
    /// Operator (row of the chart).
    std::string on;
    /// Product kind.
    std::string product;
    /// Operation name.
    std::string operation;
    /// Cycle, from 0.
    std::size_t cycle = 0U;
    /// Earliest start and end.
    Seconds start{};
    Seconds end{};
};

// ****************************************************************************
/// Result of @ref analyseWorkflow.
// ****************************************************************************
struct WorkflowAnalysis
{
    /// Period of the cell in steady state (λ).
    Seconds cycle_time{};
    /// Products made per second in steady state (products per cycle / λ).
    double throughput = 0.0;
    /// Operations of the critical circuit, as "product: operation (operator)".
    std::vector<std::string> critical;
    /// Operators of the critical circuit (@ref WorkflowOperator::id), once
    /// each: the machines to make faster or to double.
    std::vector<std::string> critical_operators;
    /// Earliest schedule of the first cycles.
    std::vector<WorkflowBar> bars;
    /// Timed event graph of the workflow, for viewers that include
    /// TimedPetriNetEditor headers (the simulator).
    std::shared_ptr<tpne::Net const> graph;
    /// Arcs of the critical circuit in @ref graph, as indexes in its arcs.
    std::vector<std::size_t> critical_arcs;
};

// ----------------------------------------------------------------------------
/// Computes the cycle time, the critical circuit and the schedule.
/// @param p_workflow Operators, products, routes.
/// @param p_cycles Cycles of the schedule (Gantt bars).
/// @return The analysis, or why it cannot be done.
// ----------------------------------------------------------------------------
[[nodiscard]] Result<WorkflowAnalysis> analyseWorkflow(WorkflowDescription const& p_workflow,
                                                       std::size_t p_cycles = 3U);

// ----------------------------------------------------------------------------
/// Marking of the event graph after some operations started (the state
/// equation of an event graph: a place has its initial tokens, plus the
/// firings of the transition before it, minus the firings of the one after).
/// @param p_graph Event graph of @ref WorkflowAnalysis::graph.
/// @param p_started Firings by transition caption (@c M1_P1), see
///   @c startedOperations (WorkflowGrafcet.hpp).
/// @return Tokens of each place, by place index.
// ----------------------------------------------------------------------------
[[nodiscard]] std::vector<std::size_t>
eventGraphMarking(tpne::Net const& p_graph, std::map<std::string, std::size_t> const& p_started);

// ----------------------------------------------------------------------------
/// Writes the workflow as a flowshop table that TimedPetriNetEditor opens
/// (@c x.flowshop: the operators in rows, the products in columns, the
/// durations in the cells).
/// @param p_workflow Workflow to write.
/// @param p_out Stream receiving the table.
/// @return What is wrong in the workflow.
// ----------------------------------------------------------------------------
[[nodiscard]] Result<void> writeFlowshop(WorkflowDescription const& p_workflow, std::ostream& p_out);

// ----------------------------------------------------------------------------
/// Mean duration of the successful runs of each skill.
/// @param p_skills Scheduler of a finished run.
/// @return Duration by skill name (skills that never succeeded are absent).
// ----------------------------------------------------------------------------
[[nodiscard]] std::map<std::string, Seconds> measuredDurations(SkillScheduler const& p_skills);

// ----------------------------------------------------------------------------
/// Replaces the declared duration of the operations by the measured duration
/// of their skill, when known.
/// @param p_workflow Workflow to change.
/// @param p_measured Durations by skill name (@ref measuredDurations).
/// @return Number of operations changed.
// ----------------------------------------------------------------------------
std::size_t useMeasuredDurations(WorkflowDescription& p_workflow,
                                 std::map<std::string, Seconds> const& p_measured);

} // namespace robotik
