// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file Action.hpp
/// What an agent decided, as a list of numbers. An adapter turns it into
/// commands for the robot.
///
/// @par Contract
/// - Promises: a flat vector; its layout is the @ref robotik::Space of the
///   environment.
#pragma once

#include <vector>

namespace robotik
{

// ****************************************************************************
/// Flat command vector produced by an @ref Agent.
///
/// The meaning of each value is set by the environment that reads it (its
/// action @ref Space): joint targets, wheel speeds, a gripper command... The
/// agent only fills numbers in the right order.
///
/// @code
/// robotik::Action action;
/// action.values = { 0.2f, -0.1f, 1.0f }; // e.g. dx, dy, close gripper
/// @endcode
// ****************************************************************************
struct Action
{
    /// Command values, in the order of the action space.
    std::vector<float> values;
};

} // namespace robotik
