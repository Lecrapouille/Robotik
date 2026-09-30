// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "Robotik/Systems/MujocoSyncSystem.hpp"

#include "Robotik/Backends/MujocoBackend.hpp"
#include "Robotik/ECS/ActuatorComponents.hpp"
#include "Robotik/ECS/BackendComponents.hpp"
#include "Robotik/ECS/JointComponents.hpp"

#include "Compages/World/Entity.hpp"

namespace robotik
{

void MujocoSyncSystem::readState(compages::world::World& p_world,
                                 MujocoBackend& p_mujoco)
{
    p_world.each<ecs::JointState, ecs::MujocoJointBinding>(
        [&](compages::world::Entity,
            ecs::JointState& p_state,
            ecs::MujocoJointBinding& p_binding)
        {
            if (p_binding.qpos_index >= 0)
            {
                p_state.position = p_mujoco.qpos(p_binding.qpos_index);
            }
            if (p_binding.qvel_index >= 0)
            {
                p_state.velocity = p_mujoco.qvel(p_binding.qvel_index);
            }
        });
}

void MujocoSyncSystem::writeCommands(compages::world::World& p_world,
                                     MujocoBackend& p_mujoco)
{
    p_mujoco.clearAppliedForces();
    p_mujoco.compensateGravity();
    p_world.each<ecs::ActuatorCommand, ecs::MujocoJointBinding>(
        [&](compages::world::Entity p_entity,
            ecs::ActuatorCommand& p_command,
            ecs::MujocoJointBinding& p_joint)
        {
            if (p_entity.has<ecs::MujocoActuatorBinding>())
            {
                int const actuator =
                    p_entity.get<ecs::MujocoActuatorBinding>().actuator_id;
                if (actuator >= 0)
                {
                    p_mujoco.setCtrl(actuator, p_command.effort);
                    return;
                }
            }
            p_mujoco.addQfrc(p_joint.dof_index, p_command.effort);
        });
}

} // namespace robotik
