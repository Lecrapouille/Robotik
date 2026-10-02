// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "Robotik/Systems/PinocchioSyncSystem.hpp"

#include "Robotik/Backends/PinocchioBackend.hpp"
#include "Robotik/ECS/BackendComponents.hpp"
#include "Robotik/ECS/JointComponents.hpp"

#include "Compages/World/Entity.hpp"

namespace robotik
{

void PinocchioSyncSystem::update(compages::world::World& p_world,
                                 PinocchioBackend& p_pinocchio) const
{
    std::vector<double> q = p_pinocchio.configuration();
    std::vector<double> v = p_pinocchio.velocity();

    p_world.each<ecs::JointState, ecs::PinocchioJointBinding>(
        [&q, &v](compages::world::Entity,
                 ecs::JointState const& p_state,
                 ecs::PinocchioJointBinding const& p_binding)
        {
            // Write the position to the configuration
            if (p_binding.q_index >= 0 &&
                p_binding.q_index < static_cast<int>(q.size()))
            {
                auto const index = static_cast<std::size_t>(p_binding.q_index);
                q[index] = ecs::positionSi(p_state);
            }

            // Write the velocity to the velocity
            if (p_binding.v_index >= 0 &&
                p_binding.v_index < static_cast<int>(v.size()))
            {
                auto const index = static_cast<std::size_t>(p_binding.v_index);
                v[index] = ecs::velocitySi(p_state);
            }
        });

    p_pinocchio.setConfiguration(q);
    p_pinocchio.setVelocity(v);
}

} // namespace robotik
