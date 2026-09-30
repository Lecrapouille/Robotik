/**
 * @file JointComponents.hpp
 * @brief Joint identity, state, commands, and limits on link entities.
 */

#pragma once

#include <string>

namespace robotik::ecs
{

/**
 * @brief How @ref JointCommand is interpreted by @ref ControllerSystem.
 */
enum class JointControlMode
{
    Disabled,  ///< No effort from the controller.
    Position,  ///< Track @ref JointCommand::position via PD.
    Velocity,  ///< Track @ref JointCommand::velocity.
    Effort     ///< Direct @ref JointCommand::effort (not used by default PD).
};

/**
 * @brief URDF joint name attached to a link entity.
 */
struct Joint
{
    /** @brief Joint name matching URDF and backends. */
    std::string name;
};

/**
 * @brief Measured joint state (filled by MuJoCo sync or hardware).
 */
struct JointState
{
    /** @brief Generalized position (rad or m). */
    double position = 0.0;

    /** @brief Generalized velocity. */
    double velocity = 0.0;

    /** @brief Measured or estimated effort. */
    double effort = 0.0;
};

/**
 * @brief High-level setpoint written by skills.
 */
struct JointCommand
{
    /** @brief Active control mode. */
    JointControlMode mode = JointControlMode::Disabled;

    /** @brief Desired position when mode is Position. */
    double position = 0.0;

    /** @brief Desired velocity when mode is Velocity. */
    double velocity = 0.0;

    /** @brief Feed-forward or direct torque when mode is Effort. */
    double effort = 0.0;
};

/**
 * @brief URDF limit block copied at load time.
 */
struct JointLimits
{
    /** @brief Lower position limit. */
    double lower = 0.0;

    /** @brief Upper position limit. */
    double upper = 0.0;

    /** @brief Maximum absolute velocity from URDF. */
    double max_velocity = 0.0;

    /** @brief Maximum absolute effort from URDF. */
    double max_effort = 0.0;
};

/**
 * @brief Default posture for @ref HomeSkill and @ref RobotRuntime::hold.
 */
struct HomePosition
{
    /** @brief Home position (rad or m). */
    double position = 0.0;
};

} // namespace robotik::ecs
