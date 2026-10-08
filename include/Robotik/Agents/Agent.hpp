// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

//! @file Agent.hpp
//! @brief Something that maps an observation to an action.
//!
//! A fly brain, a reinforcement-learning policy, a classical controller or a
//! scripted player all share this surface. None of them sees MuJoCo, Compages
//! or the files that built the world.
#pragma once

#include "Robotik/Agents/Action.hpp"
#include "Robotik/Agents/Observation.hpp"

namespace robotik
{

// ****************************************************************************
//! @brief Closed-loop decision, one step at a time.
// ****************************************************************************
class Agent
{
public:

    virtual ~Agent() = default;

    virtual Action update(Observation const& p_observation, Seconds p_dt) = 0;
};

} // namespace robotik
