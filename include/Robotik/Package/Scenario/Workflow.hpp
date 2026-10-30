// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file Workflow.hpp
/// Reads a @c x.workflow.yaml file: the operators, products and routes of a
/// cell (@ref robotik::WorkflowDescription).
///
/// @code
/// workflow:
///   id: pick_and_place_line
///   operators:                      # in the order the products visit them
///     - { id: camera, entity: robot }
///     - { id: arm, entity: robot, capacity: 1 }
///   products:
///     - id: red_cube
///       count: 1                    # products of this kind in the cell
///       route:
///         - { name: detect, on: camera, skill: "Detect(red_cube)", duration: 0.5 }
///         - { name: move, on: arm, duration: 6.0 }
/// @endcode
///
/// To generate the GRAFCET of the cell (Task/Workflow/WorkflowGrafcet.hpp),
/// an operator also has its recipes, a product its parts, and the buffers
/// and the output are given:
///
/// @code
///   operators:
///     - id: M1
///       take: [ "M1.Hover({part})", "M1.Grasp({part}) . holding(M1, {part})" ]
///       give: [ "M1.Over({to})", "M1.Release" ]
///   buffers:
///     - { id: station, after: M1, places: { P1: slot1 }, zone: true }
///   output: bin
///   products:
///     - { id: P1, parts: [p1a, p1b], route: [...] }
/// @endcode
///
/// @par Contract
/// - Promises: a route only names known operators, in the order of the
///   operators list (a flowshop), each at most once. Recipes only use the
///   placeholders @c {part}, @c {to}, @c {on}; a buffer follows a known
///   operator (one buffer per operator) and names known products.
/// - Errors: a missing or wrong field is a failure naming the file and the
///   field.
#pragma once

#include "Robotik/Common/Result.hpp"
#include "Robotik/Task/WorkflowDescription.hpp"

#include <filesystem>

namespace robotik
{

// ----------------------------------------------------------------------------
/// Reads and checks a workflow file.
/// @param p_file A @c x.workflow.yaml file.
/// @return The workflow, or what is wrong.
// ----------------------------------------------------------------------------
[[nodiscard]] Result<WorkflowDescription> loadWorkflow(std::filesystem::path const& p_file);

} // namespace robotik
