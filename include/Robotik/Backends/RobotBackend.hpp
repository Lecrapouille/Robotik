// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

//! @file RobotBackend.hpp
//! @brief What moves the joints of a robot: physics engine, kinematic model or
//! hardware driver.
#pragma once

#include "Robotik/Sensors/Measurements.hpp"

#include <optional>
#include <string>

namespace robotik
{

class Robot;

// ****************************************************************************
//! @brief Time-stepping side of a robot.
//!
//! A <em>backend</em> is a thin adapter around an external library or a
//! driver. Skills never see it: they command the @ref JointSet and the
//! actuators of the @ref Robot, and the backend applies those commands and
//! measures the joints at each @ref step. Kinematics (FK/IK) are not a
//! backend concern: every robot owns a @ref PinocchioBackend for that.
//!
//! Implementations:
//! @li @ref MujocoBackend — rigid-body dynamics with contacts (simulation);
//! @li a kinematic model — joints follow their commands (fast tests, RL);
//! @li a hardware driver — writes the commands to the motor controllers and
//!     reads the encoders.
//!
//! The optional queries (@ref raycast, @ref wrench) let simulated sensors
//! measure the world; a hardware driver leaves them empty and its sensors get
//! their data from the devices.
// ****************************************************************************
class RobotBackend
{
public:

    virtual ~RobotBackend() = default;

    // -------------------------------------------------------------------------
    //! @brief Binds the backend to the joints, actuators and sensors of
    //! @p_robot. Called once by @ref RobotSession::connect.
    // -------------------------------------------------------------------------
    virtual void attach(Robot& p_robot) = 0;

    // -------------------------------------------------------------------------
    //! @brief Puts the backend at rest on the current joint positions and
    //! base pose.
    // -------------------------------------------------------------------------
    virtual void reset(Robot& p_robot) = 0;

    // -------------------------------------------------------------------------
    //! @brief Applies the joint commands for @p_dt, then writes the measured
    //! joints (@ref JointSet::measure) and base (@ref Robot::measureBase).
    // -------------------------------------------------------------------------
    virtual void step(Robot& p_robot, Seconds p_dt) = 0;

    // -------------------------------------------------------------------------
    //! @brief Contacts after the last step (simulation only).
    // -------------------------------------------------------------------------
    [[nodiscard]] virtual int contacts() const
    {
        return 0;
    }

    // -------------------------------------------------------------------------
    //! @brief Distance to the first surface hit by a ray, ignoring the robot.
    //! @param p_origin Ray origin in the world frame (m).
    //! @param p_direction Unit direction in the world frame.
    //! @param p_max Longest distance looked at (m).
    //! @return Nothing when nothing was hit within @p_max or when the backend
    //! cannot cast rays.
    // -------------------------------------------------------------------------
    [[nodiscard]] virtual std::optional<double>
    raycast(Vector3 const& /*p_origin*/,
            Vector3 const& /*p_direction*/,
            double /*p_max*/) const
    {
        return std::nullopt;
    }

    // -------------------------------------------------------------------------
    //! @brief Wrench the link @p_link receives from its parent link, in the
    //! link frame, or nothing if unknown.
    // -------------------------------------------------------------------------
    [[nodiscard]] virtual std::optional<Wrench>
    wrench(std::string const& /*p_link*/) const
    {
        return std::nullopt;
    }
};

} // namespace robotik
