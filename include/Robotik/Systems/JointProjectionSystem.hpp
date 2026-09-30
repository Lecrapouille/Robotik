/**
 * @file JointProjectionSystem.hpp
 * @brief Pushes @ref ecs::JointState into Compages joint transforms.
 */

#pragma once

namespace compages::world
{
class World;
}

namespace robotik
{

/**
 * @brief Copies simulated joint positions onto @c RevoluteJoint / @c PrismaticJoint.
 */
class JointProjectionSystem
{
public:

    /**
     * @brief Updates Compages joint angles or offsets from @ref ecs::JointState.
     * @param p_world ECS world.
     */
    void update(compages::world::World& p_world);
};

} // namespace robotik
