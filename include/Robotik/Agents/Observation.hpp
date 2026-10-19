// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file Observation.hpp
/// What an agent is allowed to know at one instant.
///
/// A flat vector, same layout an @ref Environment writes into its observation
/// buffer. The agent never receives a camera image, a texture or a simulator
/// handle.
#pragma once

#include "Robotik/Math/Si.hpp"

#include <vector>

namespace robotik
{

// ****************************************************************************
/// Timestamped sensor vector handed to an @ref Agent.
///
/// The order of @ref values follows the observation @ref Space of the
/// environment (for example joint angles, then the target position).
///
/// @code
/// robotik::Observation obs;
/// obs.timestamp = Seconds(0.5);
/// obs.values = { q0, q1, target_x, target_y };
/// robotik::Action action = agent.update(obs, Seconds(0.01));
/// @endcode
// ****************************************************************************
struct Observation
{
    /// Simulated time when the values were measured.
    Seconds timestamp{};
    /// Measured values, in the order of the observation space.
    std::vector<float> values;
};

} // namespace robotik
