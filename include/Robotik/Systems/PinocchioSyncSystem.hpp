/**
 * @file PinocchioSyncSystem.hpp
 * @brief Copies ECS joint positions into Pinocchio configuration.
 */

#pragma once

namespace compages::world
{
class World;
}

namespace robotik
{

class PinocchioBackend;

/**
 * @brief Fills Pinocchio @c q (and optionally @c v) from @ref ecs::JointState.
 */
class PinocchioSyncSystem
{
public:

    /**
     * @brief Writes bound joint states into the Pinocchio model.
     * @param p_world ECS world.
     * @param p_pinocchio Backend to update; call @ref PinocchioBackend::updateKinematics after.
     */
    void update(compages::world::World& p_world, PinocchioBackend& p_pinocchio);
};

} // namespace robotik
