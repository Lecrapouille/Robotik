//=============================================================================
// Source: https://github.com/Robotik-Core/Robotik
// Copyright (c) 2020-2026 Quentin Quadrat.
// Distributed under MIT License.
//=============================================================================
// @file ActuatorComponents.hpp
// @brief Low-level joint control outputs and PD tuning on ECS link entities.
//=============================================================================

#pragma once

namespace robotik::ecs
{

// ****************************************************************************
//! @brief Generalized effort written to MuJoCo each physics sub-step.
//!
//! Produced by @ref ControllerSystem from commands and controller gains.
// ****************************************************************************
struct ActuatorCommand
{
    //!< Generalized force for this joint's DOF (MuJoCo @c qfrc_applied / @c
    //!< ctrl): revolute → torque (SI: N·m); prismatic → force (SI: N). Same
    //!< convention as @ref JointState::effort and URDF effort limits on @ref
    //!< JointLimits::max_effort.
    double effort = 0.0;
};

// ****************************************************************************
//! @brief Joint-space PD position loop with rate-limited setpoint.
// ****************************************************************************
struct PositionController
{
    //!< Proportional gain on position error (no units).
    double kp = 150.0;
    //!< Derivative gain on velocity (no units).
    double kd = 15.0;
    //!< Internal position setpoint slewed toward @ref JointCommand::position
    //!< (same SI as the joint: rad or m). Avoids a step in torque when the
    //!< command jumps.
    double reference = 0.0;
    //!< Dimensionless scale in @c [0, 1] on @ref JointLimits::max_velocity
    //!< (rad/s or m/s); max reference slew per step is @c speed_ratio *
    //!< max_velocity * @c dt.
    double speed_ratio = 0.5;
};

// ****************************************************************************
//! @brief Proportional velocity tracking (effort = kp * (cmd - state)).
// ****************************************************************************
struct VelocityController
{
    //!< Velocity loop gain (no units).
    double kp = 5.0;
};

} // namespace robotik::ecs
