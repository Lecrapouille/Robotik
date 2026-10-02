// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "Robotik/Systems/JointProjectionSystem.hpp"

#include "Robotik/ECS/JointComponents.hpp"

#include "Compages/World/Entity.hpp"

namespace robotik
{

void JointProjectionSystem::update(compages::world::World& p_world) const
{
    p_world.each<ecs::JointState>(
        [](compages::world::Entity p_entity, ecs::JointState const& p_state)
        {
            if (p_entity.has<compages::world::RevoluteJoint>())
            {
                p_entity.angle(
                    units::angle::radian_t(ecs::positionSi(p_state)));
            }
            else if (p_entity.has<compages::world::PrismaticJoint>())
            {
                p_entity.offset(
                    units::length::meter_t(ecs::positionSi(p_state)));
            }
        });
}

} // namespace robotik
