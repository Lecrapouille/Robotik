// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "Robotik/Systems/GraspSystem.hpp"

#include "Robotik/Scene/ContainerBounds.hpp"
#include "Robotik/ECS/ObjectComponents.hpp"
#include "Robotik/Robot/Robot.hpp"

#include "Compages/Core/Units.hpp"
#include "Compages/World/World.hpp"

#include <algorithm>
#include <optional>

namespace robotik
{

static constexpr Length kVacuumContactDistance{ 0.02 };
static constexpr Length kReleaseFloorLift{ 1e-4 };
//! Extra reach beyond the outer walls when choosing a bin to drop into.
static constexpr Length kOuterFootprintSlack{ 0.05 };

static Vector3 positionOf(compages::world::Entity p_entity)
{
    auto const at = p_entity.position();
    return { at.x, at.y, at.z };
}

//! Height of the support below @p_at: a container floor or the ground (z = 0).
static Length supportBelow(compages::world::World& p_world, Vector3 const& p_at)
{
    Length floor{ 0.0 };
    p_world.each<ecs::SceneObject>(
        [&floor, &p_at](compages::world::Entity p_entity,
                        ecs::SceneObject const& p_object)
        {
            Vector3 const center = positionOf(p_entity);
            if (std::optional<scene::ContainerInner> const inner =
                    scene::innerBounds(p_object, center))
            {
                if (scene::insideInner(*inner, p_at))
                {
                    floor = std::max(floor, Length(inner->floor_z));
                }
            }
        });
    return floor;
}

//! Final pose after vacuum off: the nearest open box under the object, or the
//! support below it (container floor, otherwise the ground).
static Vector3 settle(compages::world::World& p_world,
                      compages::world::Entity p_held,
                      Vector3 p_at)
{
    ecs::SceneObject const* shape = p_held.find<ecs::SceneObject>();
    if (shape == nullptr)
    {
        return p_at;
    }
    double const half_z = (shape->size[2] * 0.5).value();
    std::optional<Vector3> best;
    double best_distance = 0.0;
    p_world.each<ecs::SceneObject>(
        [&](compages::world::Entity p_entity, ecs::SceneObject const& p_object)
        {
            if (p_entity.id() == p_held.id())
            {
                return;
            }
            Vector3 const center = positionOf(p_entity);
            std::optional<Vector3> const pose =
                scene::dropInto(p_object,
                                center,
                                p_at,
                                half_z,
                                kReleaseFloorLift.value(),
                                kOuterFootprintSlack.value());
            if (!pose)
            {
                return;
            }
            double const dx = p_at.x - center.x;
            double const dy = p_at.y - center.y;
            double const distance = dx * dx + dy * dy;
            if (!best || distance < best_distance)
            {
                best = *pose;
                best_distance = distance;
            }
        });
    if (best)
    {
        return *best;
    }
    p_at.z = (supportBelow(p_world, p_at) + Length(half_z)).value();
    return p_at;
}

//! Nearest graspable cube whose top face is within @ref kVacuumContactDistance.
static compages::world::EntityId touched(compages::world::World& p_world,
                                           Vector3 const& p_tip)
{
    compages::world::EntityId found{};
    Length best = kVacuumContactDistance;
    p_world.each<ecs::SceneObject>(
        [&](compages::world::Entity p_entity, ecs::SceneObject const& p_object)
        {
            if (p_object.type != ecs::SceneObject::Type::CUBE)
            {
                return;
            }
            Vector3 top = positionOf(p_entity);
            top.z += (p_object.size[2] * 0.5).value();
            Length const gap{ (top - p_tip).norm() };
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
    ActuatorSet const& actuators = p_robot.actuators();
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
            Vector3 at = positionOf(held);
            at = settle(world, held, at);
            held.position(static_cast<float>(at.x),
                          static_cast<float>(at.y),
                          static_cast<float>(at.z));
        }

        if (gripper->holding())
        {
            compages::world::Entity held = world.entity(gripper->held());
            ecs::SceneObject const* shape = held.find<ecs::SceneObject>();
            if (shape == nullptr)
            {
                continue;
            }
            Length const half = shape->size[2] * 0.5;
            Vector3 const center = gripper->tip(p_robot, half);
            held.position(static_cast<float>(center.x),
                          static_cast<float>(center.y),
                          static_cast<float>(center.z));
        }
    }
}

} // namespace robotik
