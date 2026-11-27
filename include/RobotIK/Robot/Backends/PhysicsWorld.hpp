// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file PhysicsWorld.hpp
/// The simulated world around the robots: cast a ray, read a contact, the
/// pose of a free body or the force on a link, hold or move a body. It
/// exists in simulation only.
///
/// A @ref robotik::RobotBackend moves one robot and reads its joints: that
/// is the same in simulation and on hardware. Measuring the world is not:
/// a real lidar measures by itself, and nothing can weld a real box to a
/// real cup. So these queries are kept apart, in a class that a simulator
/// implements and a hardware driver does not.
///
/// @par Contract
/// - Promises: @ref robotik::RobotRuntime::getPhysics is null on hardware,
///   and the code that reads it (simulated sensors, truth, the world side
///   of a scenario) does nothing then.
/// - Expects: a body or link name of the simulated world.
/// - Errors: an unknown name answers "nothing" (@c std::nullopt, false),
///   never throws.
#pragma once

#include "RobotIK/Robot/Sensors/Measurements.hpp"

#include <optional>
#include <string>
#include <vector>

namespace robotik
{

// ****************************************************************************
/// Two bodies touching after a step.
///
/// @code
/// for (robotik::Contact const& c : physics.contactList())
///     if (c.second == robotik::Contact::GROUND)
///         std::cout << c.first << " touches the floor\n";
/// @endcode
// ****************************************************************************
struct Contact
{
    /// Body name in the simulator; @ref GROUND for the floor plane.
    std::string first;
    /// Other body name; @ref GROUND for the floor plane.
    std::string second;
    /// Contact point in the world frame (m).
    Vector3 position = Vector3::ZERO;

    /// Name of the floor plane.
    static constexpr char const* GROUND = "world";
};

// ****************************************************************************
/// What a simulator knows of the world, and the few changes a scenario may
/// make to it (see the file comment).
///
/// Who uses it: the simulated sensors (a range scanner casts rays, a
/// force/torque sensor reads the wrench of its link), the truth of the
/// @ref RoboticCell (body poses, contacts) and the grasp models (a weld
/// holds a box). Skills never see it.
///
/// @code
/// if (robotik::PhysicsWorld const* physics = runtime.getPhysics())
///     if (auto const hit = physics->raycast(origin, direction, 5.0))
///         distance = *hit;
/// @endcode
///
/// Implementations: @ref MujocoBackend (it is also the
/// @ref RobotBackend of the robots of its world).
// ****************************************************************************
class PhysicsWorld
{
public:

    virtual ~PhysicsWorld() = default;

    // -------------------------------------------------------------------------
    /// Distance to the first surface hit by a ray, ignoring the robot.
    /// @param p_origin Ray origin in the world frame (m).
    /// @param p_direction Unit direction in the world frame.
    /// @param p_max Longest distance looked at (m).
    /// @return Nothing when nothing was hit within @p_max.
    // -------------------------------------------------------------------------
    [[nodiscard]] virtual std::optional<double>
    raycast(Vector3 const& p_origin,
            Vector3 const& p_direction,
            double p_max) const = 0;

    // -------------------------------------------------------------------------
    /// Measures the wrench a link receives from its parent link.
    /// @param p_link Link name.
    /// @return The wrench in the link frame, or nothing if unknown.
    // -------------------------------------------------------------------------
    [[nodiscard]] virtual std::optional<Wrench>
    wrench(std::string const& p_link) const = 0;

    // -------------------------------------------------------------------------
    /// Reads the pose of a body.
    /// @param p_body Body name in the simulator.
    /// @return World pose after the last step, or nothing when unknown.
    // -------------------------------------------------------------------------
    [[nodiscard]] virtual std::optional<Pose>
    bodyPose(std::string const& p_body) const = 0;

    // -------------------------------------------------------------------------
    /// @return Every contact after the last step, floor included.
    // -------------------------------------------------------------------------
    [[nodiscard]] virtual std::vector<Contact> contactList() const = 0;

    // -------------------------------------------------------------------------
    /// @return The number of contacts after the last step.
    // -------------------------------------------------------------------------
    [[nodiscard]] virtual int contacts() const = 0;

    // -------------------------------------------------------------------------
    /// Turns on or off a weld declared before the simulator was built: on,
    /// the second body keeps its current pose relative to the first.
    /// @param p_name Weld name.
    /// @param p_on True to hold, false to release.
    /// @return False if there is no such weld.
    // -------------------------------------------------------------------------
    virtual bool weld(std::string const& p_name, bool p_on) = 0;

    // -------------------------------------------------------------------------
    /// Moves a free body, at rest.
    /// @param p_body Body name in the simulator.
    /// @param p_pose New world pose.
    /// @return False if there is no such free body.
    // -------------------------------------------------------------------------
    virtual bool placeBody(std::string const& p_body, Pose const& p_pose) = 0;
};

} // namespace robotik
