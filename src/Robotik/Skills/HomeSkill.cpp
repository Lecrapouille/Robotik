// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "Robotik/Skills/HomeSkill.hpp"

#include "Robotik/ECS/JointComponents.hpp"
#include "Robotik/Runtime/RobotContext.hpp"

#include "Compages/World/Entity.hpp"

namespace robotik
{

Status HomeSkill::tick(RobotContext& p_context, Seconds /*p_dt*/)
{
    bool reached = true;
    bool any = false;
    Radians const angle_tolerance(m_tolerance);
    Length const linear_tolerance(m_tolerance);
    p_context.world.each<ecs::Joint,
                         ecs::JointCommand,
                         ecs::HomePosition,
                         ecs::JointState>(
        [&reached, &any, angle_tolerance, linear_tolerance](
            compages::world::Entity,
            ecs::Joint const& p_joint,
            ecs::JointCommand& p_command,
            ecs::HomePosition const& p_home,
            ecs::JointState const& p_state)
        {
            any = true;
            ecs::setCommandMode(p_command, ecs::JointControlMode::POSITION);
            ecs::setCommandPosition(p_command, ecs::homePositionSi(p_home));

            // Check if the revolute joint is reached
            if (p_joint.mechanism == ecs::JointMechanism::Revolute)
            {
                if (ecs::exceedsTolerance(
                        std::get<ecs::RevoluteJointState>(p_state).position,
                        std::get<ecs::RevoluteHomePosition>(p_home).position,
                        angle_tolerance))
                {
                    reached = false;
                }
            }
            // Check if the linear position is reached
            else if (ecs::exceedsTolerance(
                         std::get<ecs::PrismaticJointState>(p_state).position,
                         std::get<ecs::PrismaticHomePosition>(p_home).position,
                         linear_tolerance))
            {
                reached = false;
            }
        });

    // Check if any joint is commanded
    if (!any)
    {
        return Status::FAILURE;
    }
    return reached ? Status::SUCCESS : Status::RUNNING;
}

} // namespace robotik
