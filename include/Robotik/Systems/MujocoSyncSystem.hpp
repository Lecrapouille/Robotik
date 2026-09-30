/**
 * @file MujocoSyncSystem.hpp
 * @brief Bidirectional sync between ECS joint state and MuJoCo.
 */

#pragma once

namespace compages::world
{
class World;
}

namespace robotik
{

class MujocoBackend;

/**
 * @brief Reads MuJoCo @c qpos/@c qvel into ECS and writes actuator efforts to MuJoCo.
 */
class MujocoSyncSystem
{
public:

    /**
     * @brief Pulls generalized coordinates from MuJoCo into @ref ecs::JointState.
     * @param p_world ECS world.
     * @param p_mujoco Dynamics backend after @ref MujocoBackend::step.
     */
    void readState(compages::world::World& p_world, MujocoBackend& p_mujoco);

    /**
     * @brief Applies @ref ecs::ActuatorCommand to controls and @c qfrc_applied.
     * @param p_world ECS world.
     * @param p_mujoco Dynamics backend before @ref MujocoBackend::step.
     */
    void writeCommands(compages::world::World& p_world, MujocoBackend& p_mujoco);
};

} // namespace robotik
