// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file RobotBackend.hpp
/// What moves the joints of one robot: a physics engine, a kinematic model
/// or a driver for real hardware. It is the hardware abstraction layer of
/// RobotIK: the only part that changes between a simulated robot and a
/// real one.
///
/// The robot writes commands (targets, efforts) and reads measurements. The
/// backend sits between them and the motors. Swapping it is how the same
/// skills run in simulation and on a real robot (see doc/reference/glossary.md:
/// "Backend"). What only a simulator can do (cast rays, read contacts, weld a
/// body) is not here: see @ref robotik::PhysicsWorld.
///
/// @par Contract
/// - Promises: @c attach once, @c reset at each episode, @c step at each
///   control period, called by @ref robotik::RobotRuntime.
/// - Time: in simulation, @c step(dt) moves the world by @c dt and returns
///   when it is done. On hardware, time moves alone: @c step writes the
///   commands and reads the encoders at the period of the bus, and
///   @c time() gives the stamp of that reading. A late step (the host was
///   slow) still sends the last commands and reads what is there; it does
///   not run twice to catch up. The robot clock follows @c time().
/// - Errors: a lost link, a drive fault or an emergency stop is reported by
///   @c getHealth(), never thrown inside @c step: the runtime then puts every
///   actuator in its safe state and marks its resources failed, so the
///   skills and the behavior tree see a fault (see @ref robotik::Faults).
/// - Safety: the emergency stop and the hardware limits are in the driver
///   and in the robot controller, not in the skills. A skill may stop the
///   robot; it is never what keeps people safe.
#pragma once

#include "RobotIK/Robot/Sensors/Measurements.hpp"

#include <optional>
#include <string>

namespace robotik
{

class RobotRuntime;

// ****************************************************************************
/// Hardware abstraction layer of one robot (see the file comment).
///
/// A <em>backend</em> is a thin adapter around an external library or a
/// driver. Skills never see it: they command the @ref JointSet and the
/// actuators of the @ref Robot, and the backend applies those commands and
/// measures the joints at each @ref step. Kinematics (FK/IK) are not a
/// backend concern: every robot owns a @ref Kinematics for that.
///
/// Implementations:
/// @li @ref MujocoBackend — rigid-body dynamics with contacts (simulation);
///     it is also the @ref PhysicsWorld of its world;
/// @li a kinematic model — joints follow their commands (fast tests, RL);
/// @li a hardware driver — writes the commands to the motor controllers and
///     reads the encoders; its sensors get their data from the devices.
///
/// @code
/// class PerfectJoints final : public robotik::RobotBackend
/// {
/// public:
///     void attach(robotik::RobotRuntime&) override {}
///     void reset(robotik::RobotRuntime&) override {}
///     void step(robotik::RobotRuntime& p_runtime, Seconds) override
///     {
///         // the joints reach their position targets at once
///         robotik::JointSet const& joints = p_runtime.robot().getJoints();
///         for (robotik::JointId id = 0; id < joints.size(); ++id)
///             p_runtime.measureJoint(joints.requireRevolute(id),
///                                    robotik::Radians(joints.target(id)),
///                                    robotik::AngularVelocity(0.0));
///     }
/// };
/// runtime.connect(std::make_unique<PerfectJoints>());
/// @endcode
// ****************************************************************************
class RobotBackend
{
public:

    // ************************************************************************
    /// Whether the backend can drive the robot now.
    ///
    /// A simulator is always healthy. A driver says when the link to the
    /// robot is lost, a drive is in fault, or the emergency stop is pressed.
    ///
    /// @code
    /// Health getHealth() const override
    /// {
    ///     return m_bus.alive() ? Health{} : Health{ false, "EtherCAT link lost" };
    /// }
    /// @endcode
    // ************************************************************************
    struct Health
    {
        /// True when the commands reach the motors and the readings are
        /// fresh.
        bool ok = true;
        /// Why not, in words for the operator ("EtherCAT link lost").
        std::string why;
    };

    virtual ~RobotBackend() = default;

    // -------------------------------------------------------------------------
    /// Binds the backend to the joints, actuators and sensors of a robot.
    /// Called once by @ref RobotRuntime::connect.
    /// @param p_robot Robot to drive.
    // -------------------------------------------------------------------------
    virtual void attach(RobotRuntime& p_robot) = 0;

    // -------------------------------------------------------------------------
    /// Puts the backend at rest on the current joint positions and base pose.
    /// @param p_robot Robot to drive.
    // -------------------------------------------------------------------------
    virtual void reset(RobotRuntime& p_robot) = 0;

    // -------------------------------------------------------------------------
    /// Applies the joint commands for one step, then writes the measured
    /// joints (@ref RobotRuntime::measureJoint) and base
    /// (@ref RobotRuntime::measureBase). Never throws for a problem of the
    /// robot: say it with @ref getHealth.
    /// @param p_robot Robot to drive.
    /// @param p_dt Duration of the step (simulation), or the control period
    ///   asked (hardware).
    // -------------------------------------------------------------------------
    virtual void step(RobotRuntime& p_robot, Seconds p_dt) = 0;

    // -------------------------------------------------------------------------
    /// The clock of the robot after the last @ref step. A simulator gives the
    /// time it computed (RobotIK asked for @c dt, the world waited). A
    /// hardware backend gives the stamp of its last reading: the robot clock
    /// when the robot has one, else the monotonic clock of the host
    /// (@c std::chrono::steady_clock) when the reading arrived.
    /// @return The time, or nothing: the robot then adds the @c dt of each
    ///   step to its clock.
    // -------------------------------------------------------------------------
    [[nodiscard]] virtual std::optional<Seconds> time() const
    {
        return std::nullopt;
    }

    // -------------------------------------------------------------------------
    /// @return Whether the backend drives the robot now (see @ref Health).
    ///   Read by the runtime after each @ref step.
    // -------------------------------------------------------------------------
    [[nodiscard]] virtual Health getHealth() const
    {
        return {};
    }
};

} // namespace robotik
