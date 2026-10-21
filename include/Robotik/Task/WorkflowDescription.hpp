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
/// @par Contract
/// - Promises: plain data, no behavior.
/// - Expects: checked by the reader (known operators, flowshop order).
#pragma once

#include "Robotik/Common/Math.hpp"

#include <cstdint>
#include <filesystem>
#include <string>
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
};

} // namespace robotik
