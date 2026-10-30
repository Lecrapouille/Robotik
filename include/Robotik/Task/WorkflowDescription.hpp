// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file WorkflowDescription.hpp
/// The workflow of a cell, as data: who works (operators: robots, machines,
/// cameras), what is made (products), and the route of each product (its
/// operations, on which operator, how long). It is read from a
/// @c x.workflow.yaml file (Package/Scenario/Workflow.hpp) and analysed by
/// @ref robotik::analyseWorkflow (cycle time, throughput, bottleneck).
///
/// The words come from SynDEx (INRIA): an @e operator runs operations. The
/// cell is a @e flowshop: every product visits the operators in the order of
/// @ref WorkflowDescription::operators (it may skip some).
///
/// A workflow can also say how the cell does its work, to generate the
/// GRAFCET driving it (Task/Workflow/WorkflowGrafcet.hpp): the skills of
/// each operator to take and to give a part (its @e recipe), the @e buffers
/// where parts wait between two operators, and the parts of each product
/// kind (one per lot).
///
/// @par Contract
/// - Promises: plain data, no behavior.
/// - Expects: checked by the reader (known operators, flowshop order).
#pragma once

#include "Robotik/Common/Math.hpp"

#include <cstdint>
#include <filesystem>
#include <string>
#include <utility>
#include <vector>

namespace robotik
{

// ****************************************************************************
/// Who works: a robot, a machine, a camera...
// ****************************************************************************
struct WorkflowOperator
{
    /// Name used by the routes.
    std::string id;
    /// World entity it stands for, or empty.
    std::string entity;
    /// Products it works on at once (tokens of its circuit).
    std::uint32_t capacity = 1U;
    /// Recipe to take a part: one skill call per step, optionally followed
    /// by " . " and more of the receptivity ("Grasp({part}) . holding({part})").
    /// @c {part}, @c {to} and @c {on} are replaced. Empty: nothing to do.
    std::vector<std::string> take;
    /// Recipe to give the part to the next operator, as @ref take.
    std::vector<std::string> give;
};

// ****************************************************************************
/// Where parts wait between an operator and the next one of their route.
// ****************************************************************************
struct WorkflowBuffer
{
    /// Name of the buffer.
    std::string id;
    /// Operator giving the parts (@ref WorkflowOperator::id).
    std::string after;
    /// Place of each product kind in the buffer: product id, world entity
    /// (the @c {to} of the recipe giving the part). One part per place.
    std::vector<std::pair<std::string, std::string>> places;
    /// One operator at a time in the buffer (a shared zone); the one taking
    /// parts goes first.
    bool zone = false;
};

// ****************************************************************************
/// One step of the route of a product.
// ****************************************************************************
struct WorkflowOperation
{
    /// What is done ("pick").
    std::string name;
    /// Operator doing it (@ref WorkflowOperator::id).
    std::string on;
    /// Skill doing it in a simulation, or empty: its measured duration can
    /// replace @ref duration.
    std::string skill;
    /// Declared duration.
    Seconds duration{};
};

// ****************************************************************************
/// What is made, and its route.
// ****************************************************************************
struct WorkflowProduct
{
    /// Name of the product kind.
    std::string id;
    /// Products of this kind in the cell at once (tokens of its circuit).
    std::uint32_t count = 1U;
    /// Operations in order.
    std::vector<WorkflowOperation> route;
    /// World entities of this kind, one per lot, in the order they are made.
    std::vector<std::string> parts;
};

// ****************************************************************************
/// A workflow file, read.
///
/// @code
/// for (robotik::WorkflowProduct const& product : workflow.products)
///     for (robotik::WorkflowOperation const& operation : product.route)
///         std::cout << product.id << ": " << operation.name << " on "
///                   << operation.on << '\n';
/// @endcode
// ****************************************************************************
struct WorkflowDescription
{
    /// Identifier of the workflow.
    std::string id;
    /// File it was read from.
    std::filesystem::path file;
    /// Operators, in the order the products visit them.
    std::vector<WorkflowOperator> operators;
    /// Product kinds, in the order the operators work on them.
    std::vector<WorkflowProduct> products;
    /// Buffers between the operators.
    std::vector<WorkflowBuffer> buffers;
    /// World entity receiving the finished parts (the @c {to} of the last
    /// operation), or empty.
    std::string output;
};

} // namespace robotik
