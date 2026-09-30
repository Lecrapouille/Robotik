/**
 * @file Queries.hpp
 * @brief Small ECS lookups by joint name, object name, or end effector.
 */

#pragma once

#include "Robotik/ECS/JointComponents.hpp"
#include "Robotik/ECS/ObjectComponents.hpp"
#include "Robotik/ECS/RobotComponents.hpp"

#include "Compages/World/Entity.hpp"

#include <string_view>

namespace robotik
{

/**
 * @brief Finds the link entity that carries @ref ecs::Joint with @p_name.
 * @param p_world World to search.
 * @param p_name URDF joint name.
 * @return First matching entity, or invalid handle if none.
 */
[[nodiscard]] inline compages::world::Entity
findJoint(compages::world::World& p_world, std::string_view p_name)
{
    compages::world::Entity found;
    p_world.each<ecs::Joint>(
        [&](compages::world::Entity p_entity, ecs::Joint& p_joint)
        {
            if (!found && p_joint.name == p_name)
            {
                found = p_entity;
            }
        });
    return found;
}

/**
 * @brief Finds the entity with @ref ecs::SceneObject matching @p_name.
 * @param p_world World to search.
 * @param p_name Object name from the scenario.
 * @return Matching entity, or invalid if not found.
 */
[[nodiscard]] inline compages::world::Entity
findObject(compages::world::World& p_world, std::string_view p_name)
{
    compages::world::Entity found;
    p_world.each<ecs::SceneObject>(
        [&](compages::world::Entity p_entity, ecs::SceneObject& p_object)
        {
            if (!found && p_object.name == p_name)
            {
                found = p_entity;
            }
        });
    return found;
}

/**
 * @brief Returns the first entity tagged with @ref ecs::EndEffector.
 * @param p_world World to search.
 * @return Tool link entity, or invalid if the robot has no end effector.
 */
[[nodiscard]] inline compages::world::Entity
findTool(compages::world::World& p_world)
{
    compages::world::Entity found;
    p_world.each<ecs::EndEffector>(
        [&](compages::world::Entity p_entity, ecs::EndEffector&)
        {
            if (!found)
            {
                found = p_entity;
            }
        });
    return found;
}

} // namespace robotik
