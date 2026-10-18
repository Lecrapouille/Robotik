// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file Si.hpp
/// @brief SI unit names RobotIK uses on top of nholthaus/units.
///
/// Compages exposes the unit library, not these short names. They live here
/// so a refresh of Compages does not drop @c Seconds or @c Radians.
#pragma once

#include "Compages/Core/Units.hpp"

using Seconds = units::time::second_t;
using Radians = units::angle::radian_t;
using Length = units::length::meter_t;
using LinearVelocity = units::velocity::meters_per_second_t;
using AngularVelocity = units::angular_velocity::radians_per_second_t;
using Force = units::force::newton_t;
using Torque = units::torque::newton_meter_t;

namespace robotik
{
using ::Seconds;
using ::Radians;
using ::Length;
using ::LinearVelocity;
using ::AngularVelocity;
using ::Force;
using ::Torque;
} // namespace robotik
