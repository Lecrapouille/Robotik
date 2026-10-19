// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file Agent.hpp
/// Something that maps an observation to an action.
///
/// A fly brain, a reinforcement-learning policy, a classical controller or a
/// scripted player all share this surface. None of them sees MuJoCo, Compages
/// or the files that built the world.
#pragma once

#include "Robotik/Agents/Action.hpp"
#include "Robotik/Agents/Observation.hpp"

namespace robotik
{

// ****************************************************************************
/// Closed-loop decision, one step at a time.
///
/// Derive from it and implement @ref update. The caller gives the agent what
/// it sees and the time since the last call, and applies the returned action.
///
/// @code
/// class Proportional : public robotik::Agent
/// {
/// public:
///     robotik::Action update(robotik::Observation const& p_obs, Seconds)
///     override
///     {
///         float const error = p_obs.values[1] - p_obs.values[0];
///         return robotik::Action{ { 2.0f * error } };
///     }
/// };
/// @endcode
// ****************************************************************************
class Agent
{
public:

    virtual ~Agent() = default;

    // ------------------------------------------------------------------------
    /// Decides the next action.
    /// @param p_observation What the agent sees now.
    /// @param p_dt Time since the previous call.
    /// @return The command to apply until the next call.
    // ------------------------------------------------------------------------
    virtual Action update(Observation const& p_observation, Seconds p_dt) = 0;
};

} // namespace robotik
