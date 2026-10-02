// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

//! @file MoveJointsSkill.hpp
//! @brief Multi-joint simultaneous position skill.
#pragma once

#include "Robotik/Runtime/RobotContext.hpp"
#include "Robotik/Skills/Skill.hpp"

#include <string>

namespace robotik
{

// ****************************************************************************
//! @brief Commands several joints at once; succeeds when all are within
//! tolerance.
//!
//! Each goal uses @ref Radians for revolute joints or @ref Length for prismatic
//! joints (same SI convention as @ref ecs::JointCommand::position).
// ****************************************************************************
class MoveJointsSkill final: public Skill
{
public:

    //!< Map of joint name to target position.
    using Targets = JointPosture;

    // -------------------------------------------------------------------------
    //! @brief Creates the skill with a fixed target map.
    //! @param p_targets Joint names and goal positions.
    //! @param p_angle_tolerance Success threshold on revolute joints.
    //! @param p_linear_tolerance Success threshold on prismatic joints.
    // -------------------------------------------------------------------------
    explicit MoveJointsSkill(Targets p_targets,
                             Radians p_angle_tolerance = Radians(1e-2),
                             Length p_linear_tolerance = Length(1e-2));

    Status tick(RobotContext& p_context, Seconds p_dt) override;

private:

    //!< Joint goals.
    Targets m_targets;
    //!< |error| threshold for revolute joints.
    Radians m_angle_tolerance;
    //!< |error| threshold for prismatic joints.
    Length m_linear_tolerance;
};

} // namespace robotik
