/**
 * @file ActuatorComponents.hpp
 * @brief Low-level joint control outputs and PD tuning on ECS link entities.
 */

#pragma once

namespace robotik::ecs
{

/**
 * @brief Generalized effort written to MuJoCo each physics sub-step.
 *
 * Produced by @ref ControllerSystem from commands and controller gains.
 */
struct ActuatorCommand
{
    /** @brief Joint torque or prismatic force (model units). */
    double effort = 0.0;
};

/**
 * @brief Joint-space PD position loop with rate-limited setpoint.
 */
struct PositionController
{
    /** @brief Proportional gain on position error. */
    double kp = 150.0;

    /** @brief Derivative gain on velocity. */
    double kd = 15.0;

    /**
     * @brief Internal setpoint ramped toward @ref JointCommand::position.
     *
     * Avoids a step change in torque when the command jumps far away.
     */
    double reference = 0.0;

    /** @brief Fraction of @ref JointLimits::max_velocity used to slew reference. */
    double speed_ratio = 0.5;
};

/**
 * @brief Proportional velocity tracking (effort = kp * (cmd - state)).
 */
struct VelocityController
{
    /** @brief Velocity loop gain. */
    double kp = 5.0;
};

} // namespace robotik::ecs
