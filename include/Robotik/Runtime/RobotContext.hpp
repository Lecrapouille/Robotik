/**
 * @file RobotContext.hpp
 * @brief Per-tick view passed into skills: world, backends, and timing.
 */

#pragma once

namespace compages::world
{
class World;
}

namespace robotik
{

class PinocchioBackend;
class MujocoBackend;

/**
 * @brief Everything a skill may read or write during one tick.
 *
 * Skills never talk to backends directly except through this bundle and ECS
 * components on @c world.
 */
struct RobotContext
{
    /** @brief Canonical scene graph and ECS registry. */
    compages::world::World& world;

    /** @brief Analytical kinematics (FK, Jacobians, IK). */
    PinocchioBackend& kinematics;

    /**
     * @brief MuJoCo dynamics, or null when the same skill runs on hardware.
     */
    MujocoBackend* simulation = nullptr;

    /** @brief Simulation time in seconds at the start of the tick. */
    double time = 0.0;

    /** @brief Duration of the tick in seconds. */
    double dt = 0.0;
};

} // namespace robotik
