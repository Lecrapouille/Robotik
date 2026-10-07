// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

//! @file TeachPendant.hpp
//! @brief Operator jog, waypoint recording and joint-space playback.
//!
//! Cartesian jogs are solved once with @ref Robot::inverseKinematics
//! (Pinocchio, damped least squares). Playback is a rest-to-rest quintic in
//! joint space between the recorded configurations. The pendant only writes
//! joint targets; a @ref RobotSession step is what moves the arm.
#pragma once

#include "Robotik/Math/Geometry.hpp"
#include "Robotik/Robot/Joints.hpp"
#include "Robotik/Robot/Robot.hpp"

#include <cstddef>
#include <span>
#include <string>
#include <vector>

namespace robotik
{

// ****************************************************************************
//! @brief One recorded configuration of the arm.
// ****************************************************************************
struct TeachWaypoint
{
    std::string label;
    //!< Joint positions in @ref JointId order, SI unit of each joint.
    std::vector<double> joints;
    //!< Tool pose in the robot base frame at recording.
    Pose pose{};
    //!< Duration of the move that reaches this waypoint (s).
    double duration = 2.0;
};

// ****************************************************************************
//! @brief Virtual teach pendant for one robot.
//!
//! Stateless with respect to the simulator: it does not tick the behavior
//! tree. Suspend the @ref Simulation before jogging, otherwise the mission
//! overwrites the targets.
// ****************************************************************************
class TeachPendant
{
public:

    [[nodiscard]] std::span<TeachWaypoint const> waypoints() const
    {
        return m_waypoints;
    }

    [[nodiscard]] bool playing() const
    {
        return m_playing;
    }

    //! @brief Waypoint currently approached, or -1.
    [[nodiscard]] int segment() const
    {
        return m_playing ? m_cursor : -1;
    }

    [[nodiscard]] std::string const& error() const
    {
        return m_error;
    }

    //! @brief Shift one joint by @p_delta (rad or m). Stops playback.
    bool jogJoint(Robot& p_robot, JointId p_joint, double p_delta);

    // -------------------------------------------------------------------------
    //! @brief Shift the tool in the base frame and solve IK.
    //! @param p_rotation Small rotations about the base axes X, Y, Z (rad).
    // -------------------------------------------------------------------------
    bool jogTool(Robot& p_robot,
                 Vector3 const& p_translation,
                 Vector3 const& p_rotation);

    //! @brief Store the measured joints and the current tool pose.
    std::size_t record(Robot const& p_robot,
                       std::string p_label,
                       double p_duration);

    void erase(std::size_t p_index);
    void clear();

    //! @brief Quintic from the current command to one waypoint.
    bool goTo(Robot& p_robot, std::size_t p_index);

    //! @brief Play every waypoint in order, optionally looping.
    bool play(Robot& p_robot, bool p_loop);

    //! @brief Stop playback and servo the arm where it is.
    void stop(Robot& p_robot);

    //! @brief Advance the active quintic and write joint targets.
    void update(Robot& p_robot, Seconds p_dt);

private:

    struct Polynomial
    {
        double a0 = 0.0;
        double a3 = 0.0;
        double a4 = 0.0;
        double a5 = 0.0;

        [[nodiscard]] double position(double p_t) const;
    };

    void begin(Robot const& p_robot, TeachWaypoint const& p_goal);
    void apply(Robot& p_robot, std::vector<double> const& p_positions);
    [[nodiscard]] std::vector<double> command(Robot const& p_robot) const;
    [[nodiscard]] bool compatible(Robot const& p_robot,
                                  TeachWaypoint const& p_waypoint) const;

    std::vector<TeachWaypoint> m_waypoints;
    std::vector<Polynomial> m_polynomials;
    double m_time = 0.0;
    double m_duration = 0.0;
    bool m_playing = false;
    bool m_sequence = false;
    bool m_loop = false;
    int m_cursor = -1;
    std::string m_error;
};

} // namespace robotik
