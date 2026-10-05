// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "Robotik/Systems/GraspSystem.hpp"

#include "Robotik/ECS/ObjectComponents.hpp"
#include "Robotik/Robot/Robot.hpp"

#include "Compages/World/World.hpp"

#include <cmath>
#include <limits>

#define CONTACT_DISTANCE_M 0.02

namespace robotik
{

static Vector3 positionOf(compages::world::Entity p_entity)
{
    auto const at = p_entity.position();
    return { at.x, at.y, at.z };
}

//! @brief Height of the support below @p_at: a container floor or the ground.
static double supportBelow(compages::world::World& p_world, Vector3 const& p_at)
{
    double floor = 0.0;
    p_world.each<ecs::SceneObject>(
        [&floor, &p_at](compages::world::Entity p_entity,
                        ecs::SceneObject const& p_object)
        {
            Vector3 const center = positionOf(p_entity);
            if (p_object.type == ecs::SceneObject::Type::BOX &&
                std::abs(p_at.x - center.x) < p_object.size[0].value() * 0.5 &&
                std::abs(p_at.y - center.y) < p_object.size[1].value() * 0.5)
            {
                floor = center.z - p_object.size[2].value() * 0.5;
            }
        });
    return floor;
}

static compages::world::EntityId touched(compages::world::World& p_world,
                                         Vector3 const& p_tip)
{
    compages::world::EntityId found{};
    double best = CONTACT_DISTANCE_M;
    p_world.each<ecs::SceneObject>(
        [&](compages::world::Entity p_entity, ecs::SceneObject const& p_object)
        {
            if (p_object.type != ecs::SceneObject::Type::CUBE)
            {
                return;
            }
            Vector3 top = positionOf(p_entity);
            top.z += p_object.size[2].value() * 0.5;
            double const gap = (top - p_tip).norm();
            if (gap <= best)
            {
                best = gap;
                found = p_entity.id();
            }
        });
    return found;
}

void GraspSystem::update(Robot& p_robot) const
{
    compages::world::World& world = p_robot.world();
    ActuatorSet& actuators = p_robot.actuators();
    for (std::size_t i = 0; i < actuators.size(); ++i)
    {
        auto* gripper = dynamic_cast<VacuumGripper*>(&actuators[i]);
        if (gripper == nullptr)
        {
            continue;
        }

        if (gripper->holding() && !world.alive(gripper->held()))
        {
            gripper->held({});
        }

        if (gripper->suction() && !gripper->holding())
        {
            gripper->held(touched(world, gripper->tip(p_robot)));
        }
        else if (!gripper->suction() && gripper->holding())
        {
            compages::world::Entity held = world.entity(gripper->held());
            gripper->held({});
            Vector3 const at = positionOf(held);
            double const z = supportBelow(world, at) +
                             held.get<ecs::SceneObject>().size[2].value() * 0.5;
            held.position(static_cast<float>(at.x),
                          static_cast<float>(at.y),
                          static_cast<float>(z));
        }

        if (gripper->holding())
        {
            compages::world::Entity held = world.entity(gripper->held());
            double const half = held.get<ecs::SceneObject>().size[2].value() * 0.5;
            Vector3 const center = gripper->tip(p_robot, half);
            held.position(static_cast<float>(center.x),
                          static_cast<float>(center.y),
                          static_cast<float>(center.z));
        }
    }
}

} // namespace robotik
