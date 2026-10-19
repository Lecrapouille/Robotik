// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file Math.hpp
/// Physical units, positions and rotations, as RobotIK names them.
///
/// Nothing is defined here: the types come from Compages, the 3D library
/// RobotIK draws and stores its world with, so a robot and its picture use
/// the same numbers. This header only gives them short names in the
/// @c robotik namespace.
///
/// A unit is part of the type: a duration cannot be added to a length, and
/// a function asking for an angle cannot be given metres by mistake.
///
/// @code
/// robotik::Seconds const dt(0.01);
/// robotik::Pose tool;
/// tool.position = robotik::Vector3(0.3, 0.0, 0.2);
/// tool.rotation = robotik::Quaternion::fromRpy(robotik::Radians(0.0),
///                                              robotik::Radians(1.5708),
///                                              robotik::Radians(0.0));
/// double const heading = robotik::yaw(tool.rotation).value();
/// double const reach = robotik::norm(tool.position);
/// @endcode
#pragma once

#include "Compages/Core/Pose.hpp"
#include "Compages/Core/Units.hpp"

namespace robotik
{

using compages::si::AngularAcceleration; ///< Turn acceleration (rad/s²).
using compages::si::AngularVelocity;     ///< Turn rate (rad/s).
using compages::si::Force;               ///< Force (N).
using compages::si::Length;              ///< Distance (m).
using compages::si::LinearVelocity;      ///< Speed (m/s).
using compages::si::Radians;             ///< Angle (rad).
using compages::si::Seconds;             ///< Duration (s).
using compages::si::Torque;              ///< Torque (N.m).

/// Position (m) or direction, in double precision.
using Vector3 = compages::core::Vector3g;
/// Rotation as a unit quaternion (w, x, y, z).
using Quaternion = compages::core::Quatd;
/// Where a frame is and how it is turned, in its parent frame (position in m).
using Pose = compages::core::Posed;

using compages::core::yaw;          ///< Heading about Z of a rotation.
using compages::core::vector::norm; ///< Length of a vector.

} // namespace robotik
