// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "Robotik/Systems/ControllerSystem.hpp"

#include "Robotik/ECS/ActuatorComponents.hpp"
#include "Robotik/ECS/JointComponents.hpp"

#include "Compages/World/Entity.hpp"

#include <algorithm>

namespace robotik
{

void ControllerSystem::update(compages::world::World& p_world, double p_dt)
{
    p_world.each<ecs::JointState,
                 ecs::JointCommand,
                 ecs::PositionController,
                 ecs::ActuatorCommand>(
        [p_dt](compages::world::Entity p_entity,
               ecs::JointState& p_state,
               ecs::JointCommand& p_command,
               ecs::PositionController& p_controller,
               ecs::ActuatorCommand& p_output)
        {
            if (p_command.mode != ecs::JointControlMode::Position)
            {
                p_controller.reference = p_state.position;
                p_output.effort = 0.0;
                return;
            }

            ecs::JointLimits const* limits = p_entity.find<ecs::JointLimits>();
            double const speed = (limits != nullptr && limits->max_velocity > 0.0)
                                     ? limits->max_velocity * p_controller.speed_ratio
                                     : 1.0;
            double const step = speed * p_dt;
            p_controller.reference +=
                std::clamp(p_command.position - p_controller.reference, -step, step);

            double effort = p_controller.kp * (p_controller.reference - p_state.position) -
                            p_controller.kd * p_state.velocity;
            if (limits != nullptr && limits->max_effort > 0.0)
            {
                effort = std::clamp(effort, -limits->max_effort, limits->max_effort);
            }
            p_output.effort = effort;
        });
}

} // namespace robotik
