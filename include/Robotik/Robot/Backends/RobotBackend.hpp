// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file RobotBackend.hpp
/// What moves the joints of a robot: a physics engine, a kinematic model or a
/// driver for real hardware.
///
/// The robot writes commands (targets, efforts) and reads measurements. The
/// backend sits between them and the world. Swapping it is how the same
/// skills run in simulation and on a real robot (see doc/Glossary.md:
/// "Backend").
///
/// @par Contract
/// - Promises: @c attach once, @c reset at each episode, @c step at each
///   control period. The optional queries (contacts, body poses, wrench) answer
///   "nothing" when the backend cannot measure them.
/// - Expects: the backend reads the joint commands of the robot and writes the
///   measured joint state back; it owns no skill logic.
#pragma once

#include "Robotik/Robot/Sensors/Measurements.hpp"

#include <optional>
#include <string>
#include <vector>

namespace robotik
{

class Robot;

// ****************************************************************************
/// Two bodies touching after a step.
///
/// @code
/// for (robotik::Contact const& c : backend.contactList())
///     if (c.second == robotik::Contact::GROUND)
///         std::cout << c.first << " touches the floor\n";
/// @endcode
// ****************************************************************************
struct Contact
{
    /// Body name in the backend; @ref GROUND for the floor plane.
    std::string first;
    /// Other body name; @ref GROUND for the floor plane.
    std::string second;
    /// Contact point in the world frame (m).
    Vector3 position = Vector3::ZERO;

    /// Name of the floor plane.
    static constexpr char const* GROUND = "world";
};

// ****************************************************************************
/// Time-stepping side of a robot.
///
/// A <em>backend</em> is a thin adapter around an external library or a
/// driver. Skills never see it: they command the @ref JointSet and the
/// actuators of the @ref Robot, and the backend applies those commands and
/// measures the joints at each @ref step. Kinematics (FK/IK) are not a
/// backend concern: every robot owns a @ref PinocchioBackend for that.
///
/// Implementations:
/// @li @ref MujocoBackend — rigid-body dynamics with contacts (simulation);
/// @li a kinematic model — joints follow their commands (fast tests, RL);
/// @li a hardware driver — writes the commands to the motor controllers and
///     reads the encoders.
///
/// The optional queries (@ref raycast, @ref wrench) let simulated sensors
/// measure the world; a hardware driver leaves them empty and its sensors get
/// their data from the devices.
///
/// @code
/// class PerfectJoints final : public robotik::RobotBackend
/// {
/// public:
///     void attach(robotik::Robot&) override {}
///     void reset(robotik::Robot&) override {}
///     void step(robotik::Robot& p_robot, Seconds) override
///     {
///         // the joints reach their position targets at once
///         robotik::JointSet& joints = p_robot.joints();
///         for (robotik::JointId id = 0; id < joints.size(); ++id)
///             joints.measure(id, joints.target(id), 0.0);
///     }
/// };
/// session.connect(std::make_unique<PerfectJoints>());
/// @endcode
// ****************************************************************************
class RobotBackend
{
public:

    virtual ~RobotBackend() = default;

    // -------------------------------------------------------------------------
    /// Binds the backend to the joints, actuators and sensors of a robot.
    /// Called once by @ref RobotSession::connect.
    /// @param p_robot Robot to drive.
    // -------------------------------------------------------------------------
    virtual void attach(Robot& p_robot) = 0;

    // -------------------------------------------------------------------------
    /// Puts the backend at rest on the current joint positions and base pose.
    /// @param p_robot Robot to drive.
    // -------------------------------------------------------------------------
    virtual void reset(Robot& p_robot) = 0;

    // -------------------------------------------------------------------------
    /// Applies the joint commands for one step, then writes the measured
    /// joints (@ref JointSet::measure) and base (@ref Robot::measureBase).
    /// @param p_robot Robot to drive.
    /// @param p_dt Duration of the step.
    // -------------------------------------------------------------------------
    virtual void step(Robot& p_robot, Seconds p_dt) = 0;

    // -------------------------------------------------------------------------
    /// @return The number of contacts after the last step (simulation only).
    // -------------------------------------------------------------------------
    [[nodiscard]] virtual int contacts() const
    {
        return 0;
    }

    // -------------------------------------------------------------------------
    /// Ground-truth queries build on it; skills never see it.
    /// @return Every contact after the last step, floor included
    ///   (simulation only).
    // -------------------------------------------------------------------------
    [[nodiscard]] virtual std::vector<Contact> contactList() const
    {
        return {};
    }

    // -------------------------------------------------------------------------
    /// Reads the pose of a body (simulation only).
    /// @param p_body Body name in the backend.
    /// @return World pose after the last step, or nothing when unknown.
    // -------------------------------------------------------------------------
    [[nodiscard]] virtual std::optional<Pose>
    bodyPose(std::string const& /*p_body*/) const
    {
        return std::nullopt;
    }

    // -------------------------------------------------------------------------
    /// Moves a free body, at rest (simulation only).
    /// @param p_body Body name in the backend.
    /// @param p_pose New world pose.
    /// @return False if the backend has no such free body.
    // -------------------------------------------------------------------------
    virtual bool placeBody(std::string const& /*p_body*/,
                           Pose const& /*p_pose*/)
    {
        return false;
    }

    // -------------------------------------------------------------------------
    /// Turns on or off a weld declared before the backend was built: on, the
    /// second body keeps its current pose relative to the first.
    /// @param p_name Weld name.
    /// @param p_on True to hold, false to release.
    /// @return False if the backend has no such weld.
    // -------------------------------------------------------------------------
    virtual bool weld(std::string const& /*p_name*/, bool /*p_on*/)
    {
        return false;
    }

    // -------------------------------------------------------------------------
    /// Distance to the first surface hit by a ray, ignoring the robot.
    /// @param p_origin Ray origin in the world frame (m).
    /// @param p_direction Unit direction in the world frame.
    /// @param p_max Longest distance looked at (m).
    /// @return Nothing when nothing was hit within @p_max or when the backend
    ///   cannot cast rays.
    // -------------------------------------------------------------------------
    [[nodiscard]] virtual std::optional<double>
    raycast(Vector3 const& /*p_origin*/,
            Vector3 const& /*p_direction*/,
            double /*p_max*/) const
    {
        return std::nullopt;
    }

    // -------------------------------------------------------------------------
    /// Measures the wrench a link receives from its parent link.
    /// @param p_link Link name.
    /// @return The wrench in the link frame, or nothing if unknown.
    // -------------------------------------------------------------------------
    [[nodiscard]] virtual std::optional<Wrench>
    wrench(std::string const& /*p_link*/) const
    {
        return std::nullopt;
    }
};

} // namespace robotik
