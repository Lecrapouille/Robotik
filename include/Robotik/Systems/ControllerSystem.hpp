/**
 * @file ControllerSystem.hpp
 * @brief Maps @ref ecs::JointCommand to @ref ecs::ActuatorCommand via PD loops.
 */

#pragma once

namespace compages::world
{
class World;
}

namespace robotik
{

/**
 * @brief Writes actuator efforts from joint commands and controller components.
 *
 * Position mode uses rate-limited reference tracking on @ref ecs::PositionController.
 */
class ControllerSystem
{
public:

    /**
     * @brief Updates all entities with joint command and controller components.
     * @param p_world ECS world.
     * @param p_dt Timestep used to slew position references.
     */
    void update(compages::world::World& p_world, double p_dt);
};

} // namespace robotik
