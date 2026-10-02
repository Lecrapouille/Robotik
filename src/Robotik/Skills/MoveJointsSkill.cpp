// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "Robotik/Skills/MoveJointsSkill.hpp"

#include "Robotik/ECS/JointComponents.hpp"
#include "Robotik/ECS/Queries.hpp"
#include "Robotik/Runtime/RobotContext.hpp"

namespace robotik
{

MoveJointsSkill::MoveJointsSkill(Targets p_targets,
                                 Radians p_angle_tolerance,
                                 Length p_linear_tolerance)
    : m_targets(std::move(p_targets)),
      m_angle_tolerance(p_angle_tolerance),
      m_linear_tolerance(p_linear_tolerance)
{
}

Status MoveJointsSkill::tick(RobotContext& p_context, Seconds /*p_dt*/)
{
    if (m_targets.empty())
    {
        return Status::FAILURE;
    }

    bool reached = true;
    for (auto const& [name, goal] : m_targets)
    {
        // Find the joint
        compages::world::Entity joint = findJoint(p_context.world, name);
        if (!joint || !joint.has<ecs::Joint>() ||
            !joint.has<ecs::JointCommand>() || !joint.has<ecs::JointState>())
        {
            return Status::FAILURE;
        }

        // Set the command mode and position
        ecs::JointCommand& command = joint.get<ecs::JointCommand>();
        ecs::setCommandMode(command, ecs::JointControlMode::POSITION);
        ecs::setCommandPosition(command, jointGoalSi(goal));

        // Check if the position is reached
        ecs::JointState const& state = joint.get<ecs::JointState>();
        if (exceedsJointGoalTolerance(joint.get<ecs::Joint>().mechanism,
                                      state,
                                      goal,
                                      m_angle_tolerance,
                                      m_linear_tolerance))
        {
            reached = false;
        }
    }

    return reached ? Status::SUCCESS : Status::RUNNING;
}

} // namespace robotik
