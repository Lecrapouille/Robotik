// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

//! @file GripperSkills.hpp
//! @brief Open and close skills for @ref ecs::Gripper finger joints.
#pragma once

#include "Robotik/Skills/Skill.hpp"

namespace robotik
{

// ****************************************************************************
//! @brief Commands all @ref ecs::Gripper entities to @ref
//! ecs::Gripper::max_opening.
// ****************************************************************************
class OpenGripperSkill final: public Skill
{
public:

    // -------------------------------------------------------------------------
    //! @brief Creates the skill.
    //! @param p_tolerance Max |error| on finger opening for success (SI: m).
    // -------------------------------------------------------------------------
    explicit OpenGripperSkill(Length p_tolerance = Length(1e-3))
        : m_tolerance(p_tolerance)
    {
    }

    Status tick(RobotContext& p_context, Seconds p_dt) override;

private:

    //!< Opening error threshold.
    Length m_tolerance;
};

// ****************************************************************************
//! @brief Commands all @ref ecs::Gripper entities to @ref
//! ecs::Gripper::min_opening.
// ****************************************************************************
class CloseGripperSkill final: public Skill
{
public:

    // -------------------------------------------------------------------------
    //! @brief Creates the skill.
    //! @param p_tolerance Max |error| on finger opening for success (SI: m).
    // -------------------------------------------------------------------------
    explicit CloseGripperSkill(Length p_tolerance = Length(1e-3))
        : m_tolerance(p_tolerance)
    {
    }

    Status tick(RobotContext& p_context, Seconds p_dt) override;

private:

    //!< Opening error threshold.
    Length m_tolerance;
};

} // namespace robotik
