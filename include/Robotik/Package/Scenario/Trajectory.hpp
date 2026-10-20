// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file Trajectory.hpp
/// A path the robot follows, kept in its own file and named by the scenario.
///
/// @code
/// # Cartesian: tool work point poses in the frame of a world entity.
/// trajectory:
///   type: cartesian
///   frame: easel.canvas_mount          # entity or entity.link; world if empty
///   orientation: { rpy: [1.5708, 0.0, -1.5708] }   # default for every point
///   points:
///     - [0.18, 0.12, 0.16]             # a position, default orientation
///     - { position: [0.06, 0.08, 0.16], rpy: [1.5708, 0.0, -1.5708] }
///
/// # Joint: configurations of named joints (rad or m).
/// trajectory:
///   type: joint
///   joints: [joint1, joint2]
///   points:
///     - [0.0, 0.3]
///     - [0.5, 0.3]
/// @endcode
///
/// The scenario names them, relative to the package root:
/// @code
/// scenario:
///   trajectories:
///     red: scenarios/trajectories/red_stroke.trajectory.yaml
/// @endcode
#pragma once

#include "Robotik/Task/CellDescription.hpp"

#include <filesystem>

namespace robotik
{

// ----------------------------------------------------------------------------
/// Reads a @c x.trajectory.yaml file.
/// @param p_path File to read.
/// @return The parsed trajectory.
/// @throws std::runtime_error naming the file and the field at fault.
// ----------------------------------------------------------------------------
[[nodiscard]] Trajectory loadTrajectory(std::filesystem::path const& p_path);

} // namespace robotik
