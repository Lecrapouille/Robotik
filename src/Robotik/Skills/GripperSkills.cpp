// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "Robotik/Skills/GripperSkills.hpp"

#include "Robotik/ECS/JointComponents.hpp"
#include "Robotik/ECS/RobotComponents.hpp"
#include "Robotik/Runtime/RobotContext.hpp"

#include "Compages/World/Entity.hpp"

#include <cmath>

namespace robotik
{

static Status
commandGrippers(RobotContext& p_context, bool p_open, Length p_tolerance)
{
    bool reached = true;
    bool any = false;

    p_context.world.each<ecs::Gripper, ecs::JointCommand, ecs::JointState>(
        [&p_open, &reached, &any, p_tolerance](compages::world::Entity,
                                               ecs::Gripper const& p_gripper,
                                               ecs::JointCommand& p_command,
                                               ecs::JointState const& p_state)
        {
            any = true;

            // Set the command mode and position
            double const goal =
                (p_open ? p_gripper.max_opening : p_gripper.min_opening)
                    .value();
            ecs::setCommandMode(p_command, ecs::JointControlMode::POSITION);
            ecs::setCommandPosition(p_command, goal);

            // Check if the position is reached
            if (ecs::exceedsTolerance(
                    std::get<ecs::PrismaticJointState>(p_state).position,
                    Length(goal),
                    p_tolerance))
            {
                reached = false;
            }
        });

    // Check if any gripper is commanded
    if (!any)
    {
        return Status::FAILURE;
    }
    return reached ? Status::SUCCESS : Status::RUNNING;
}

Status OpenGripperSkill::tick(RobotContext& p_context, Seconds /*p_dt*/)
{
    return commandGrippers(p_context, true, m_tolerance);
}

Status CloseGripperSkill::tick(RobotContext& p_context, Seconds /*p_dt*/)
{
    return commandGrippers(p_context, false, m_tolerance);
}

} // namespace robotik
