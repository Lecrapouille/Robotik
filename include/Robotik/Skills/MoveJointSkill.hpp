// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/**
 * @file MoveJointSkill.hpp
 * @brief Single-joint position skill.
 */

#pragma once

#include "Robotik/Skills/Skill.hpp"

#include <string>

namespace robotik
{

/**
 * @brief Drives one named joint to a fixed position setpoint.
 */
class MoveJointSkill final: public Skill
{
public:

    /**
     * @brief Creates the skill with an initial goal.
     * @param p_joint_name URDF joint name.
     * @param p_target Desired position (rad or m).
     * @param p_tolerance Success threshold on |error|.
     */
    MoveJointSkill(std::string p_joint_name,
                   double p_target,
                   double p_tolerance = 1e-2);

    /**
     * @brief Updates goal without resetting tick state.
     * @param p_joint_name Joint to move.
     * @param p_target New setpoint.
     */
    void setGoal(std::string p_joint_name, double p_target);

    /** @brief Active joint name. */
    [[nodiscard]] std::string const& jointName() const
    {
        return m_joint_name;
    }

    /** @brief Current position setpoint. */
    [[nodiscard]] double target() const
    {
        return m_target;
    }

    Status tick(RobotContext& p_context, double p_dt) override;

private:

    /** @brief Joint controlled by this skill. */
    std::string m_joint_name;

    /** @brief Commanded position. */
    double m_target = 0.0;

    /** @brief Success tolerance. */
    double m_tolerance = 1e-2;
};

} // namespace robotik
