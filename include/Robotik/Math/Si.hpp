// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file Si.hpp
/// SI unit names RobotIK uses on top of nholthaus/units.
///
/// Compages exposes the unit library, not these short names. They live here
/// so a refresh of Compages does not drop @c Seconds or @c Radians.
///
/// A typed value cannot be mixed with another unit by mistake: adding a
/// @c Length to a @c Seconds does not compile.
///
/// @code
/// Seconds dt(0.01);
/// Radians angle(1.57);
/// double raw = dt.value(); // back to a plain number when needed
/// @endcode
#pragma once

#include "Compages/Core/Units.hpp"

using Seconds = units::time::second_t;                       ///< Time [s].
using Radians = units::angle::radian_t;                      ///< Angle [rad].
using Length = units::length::meter_t;                       ///< Distance [m].
using LinearVelocity = units::velocity::meters_per_second_t; ///< Speed [m/s].
using AngularVelocity =
    units::angular_velocity::radians_per_second_t; ///< Rotation speed [rad/s].
using Force = units::force::newton_t;              ///< Force [N].
using Torque = units::torque::newton_meter_t;      ///< Torque [N.m].

namespace robotik
{
using ::AngularVelocity;
using ::Force;
using ::Length;
using ::LinearVelocity;
using ::Radians;
using ::Seconds;
using ::Torque;
} // namespace robotik
