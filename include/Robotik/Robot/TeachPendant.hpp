// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file TeachPendant.hpp
/// Operator jog, waypoint recording and joint-space playback.
///
/// Cartesian jogs are solved once with @ref robotik::Robot::inverseKinematics
/// (Pinocchio, damped least squares). Playback is a rest-to-rest quintic in
/// joint space between the recorded configurations. The pendant only writes
/// joint targets; a @ref robotik::RobotSession step is what moves the arm.
///
/// @par Contract
/// - Promises: the pendant only writes joint targets; it never steps the
///   physics.
/// - Expects: a @ref robotik::RobotSession steps after each command.
/// - Errors: a Cartesian jog that the IK cannot reach returns @c false and
///   leaves the targets as they were.
#pragma once

#include "Robotik/Common/Math.hpp"
#include "Robotik/Robot/Joints.hpp"
#include "Robotik/Robot/Robot.hpp"

#include <cstddef>
#include <span>
#include <string>
#include <utility>
#include <vector>

namespace robotik
{

// ****************************************************************************
/// One recorded configuration of the arm.
///
/// @code
/// robotik::TeachWaypoint const& w = pendant.waypoints()[0];
/// std::cout << w.label << ": " << w.joints.size() << " joints, "
///           << w.duration << " s\n";
/// @endcode
// ****************************************************************************
struct TeachWaypoint
{
    /// Name given by the operator.
    std::string label;
    /// Joint positions in @ref JointId order, SI unit of each joint.
    std::vector<double> joints;
    /// Tool pose in the robot base frame at recording.
    Pose pose{};
    /// Duration of the move that reaches this waypoint (s).
    double duration = 2.0;
};

// ****************************************************************************
/// Virtual teach pendant for one robot.
///
/// Stateless with respect to the simulator: it does not tick the behavior
/// tree. Suspend the @ref Simulation before jogging, otherwise the mission
/// overwrites the targets.
///
/// Every command returns false on failure and leaves the reason in
/// @ref error.
///
/// @code
/// robotik::TeachPendant pendant;
/// pendant.jogTool(robot, robotik::Vector3(0.0, 0.0, 0.01), robotik::Vector3::ZERO);
/// // up 1 cm pendant.record(robot, "above", 2.0); pendant.jogJoint(robot,
/// wrist, 0.2); pendant.record(robot, "turned", 1.0); pendant.play(robot,
/// true); while (pendant.playing()) {
///     pendant.update(robot, dt);
///     session.step(dt);
/// }
/// @endcode
// ****************************************************************************
class TeachPendant
{
public:

    // -------------------------------------------------------------------------
    /// @return The recorded waypoints, in order.
    // -------------------------------------------------------------------------
    [[nodiscard]] std::span<TeachWaypoint const> waypoints() const
    {
        return m_waypoints;
    }

    // -------------------------------------------------------------------------
    /// @return True while a move or a playback is in progress.
    // -------------------------------------------------------------------------
    [[nodiscard]] bool playing() const
    {
        return m_playing;
    }

    // -------------------------------------------------------------------------
    /// @return The index of the waypoint currently approached, or -1.
    // -------------------------------------------------------------------------
    [[nodiscard]] int segment() const
    {
        return m_playing ? m_cursor : -1;
    }

    // -------------------------------------------------------------------------
    /// @return Why the last command failed.
    // -------------------------------------------------------------------------
    [[nodiscard]] std::string const& error() const
    {
        return m_error;
    }

    // -------------------------------------------------------------------------
    /// Chooses the frame moved by @ref jogTool and stored by @ref record — any
    /// link of the robot (a foot, a leg tip...).
    /// @param p_frame Link name. Empty: the work frame of the robot
    ///   (@ref Robot::tool).
    // -------------------------------------------------------------------------
    void frame(std::string p_frame)
    {
        m_frame = std::move(p_frame);
    }

    // -------------------------------------------------------------------------
    /// @param p_robot Robot driven by the pendant.
    /// @return The frame moved by the pendant: the one chosen, or the tool
    ///   of @p_robot.
    // -------------------------------------------------------------------------
    [[nodiscard]] std::string const& frame(Robot const& p_robot) const;

    // -------------------------------------------------------------------------
    /// Shifts one joint. Stops playback.
    /// @param p_robot Robot to move.
    /// @param p_joint Joint to shift.
    /// @param p_delta Shift (rad or m).
    /// @return False on an unknown joint.
    // -------------------------------------------------------------------------
    bool jogJoint(Robot& p_robot, JointId p_joint, double p_delta);

    // -------------------------------------------------------------------------
    /// Shifts the tool in the base frame and solves IK. Stops playback.
    /// @param p_robot Robot to move.
    /// @param p_translation Shift along the base axes X, Y, Z (m).
    /// @param p_rotation Small rotations about the base axes X, Y, Z (rad).
    /// @return False when the robot has no tool frame or IK fails.
    // -------------------------------------------------------------------------
    bool jogTool(Robot& p_robot,
                 Vector3 const& p_translation,
                 Vector3 const& p_rotation);

    // -------------------------------------------------------------------------
    /// Stores the measured joints and the current tool pose.
    /// @param p_robot Robot to read.
    /// @param p_label Name of the waypoint; empty gives "wp N".
    /// @param p_duration Duration of the move that will reach it (s), at
    ///   least 0.05 s.
    /// @return Index of the new waypoint.
    // -------------------------------------------------------------------------
    std::size_t
    record(Robot const& p_robot, std::string p_label, double p_duration);

    // -------------------------------------------------------------------------
    /// Removes one waypoint. Stops playback.
    /// @param p_index Index of the waypoint; out of range does nothing.
    // -------------------------------------------------------------------------
    void erase(std::size_t p_index);

    /// Removes every waypoint.
    void clear();

    // -------------------------------------------------------------------------
    /// Starts a quintic move from the current command to one waypoint.
    /// @param p_robot Robot to move.
    /// @param p_index Index of the waypoint.
    /// @return False when there is no such waypoint, or it was recorded on
    ///   another robot.
    // -------------------------------------------------------------------------
    bool goTo(Robot& p_robot, std::size_t p_index);

    // -------------------------------------------------------------------------
    /// Plays every waypoint in order.
    /// @param p_robot Robot to move.
    /// @param p_loop True to start again after the last one.
    /// @return False when there is no waypoint, or one was recorded on
    ///   another robot.
    // -------------------------------------------------------------------------
    bool play(Robot& p_robot, bool p_loop);

    // -------------------------------------------------------------------------
    /// Stops playback and servos the arm where it is.
    /// @param p_robot Robot to stop.
    // -------------------------------------------------------------------------
    void stop(Robot& p_robot);

    // -------------------------------------------------------------------------
    /// Advances the active quintic and writes joint targets. Call it every
    /// step while @ref playing.
    /// @param p_robot Robot to move.
    /// @param p_dt Duration of the step.
    // -------------------------------------------------------------------------
    void update(Robot& p_robot, Seconds p_dt);

private:

    // -------------------------------------------------------------------------
    /// Rest-to-rest quintic of one joint: a0 + a3 t³ + a4 t⁴ + a5 t⁵ (no
    /// speed nor acceleration at both ends).
    // -------------------------------------------------------------------------
    struct Polynomial
    {
        double a0 = 0.0; ///< Start position.
        double a3 = 0.0; ///< Coefficient of t³.
        double a4 = 0.0; ///< Coefficient of t⁴.
        double a5 = 0.0; ///< Coefficient of t⁵.

        /// @return The position at time @p_t from the start of the move.
        [[nodiscard]] double position(double p_t) const;
    };

    /// Computes one quintic per joint toward @p_goal.
    void begin(Robot const& p_robot, TeachWaypoint const& p_goal);
    /// Writes joint targets.
    void apply(Robot& p_robot, std::vector<double> const& p_positions);
    /// Current joint targets of the robot.
    [[nodiscard]] std::vector<double> command(Robot const& p_robot) const;
    /// True when @p_waypoint has one value per joint of @p_robot.
    [[nodiscard]] bool compatible(Robot const& p_robot,
                                  TeachWaypoint const& p_waypoint) const;

    /// Recorded waypoints.
    std::vector<TeachWaypoint> m_waypoints;
    /// One quintic per joint for the active move.
    std::vector<Polynomial> m_polynomials;
    /// Time since the start of the active move (s).
    double m_time = 0.0;
    /// Duration of the active move (s).
    double m_duration = 0.0;
    /// A move is in progress.
    bool m_playing = false;
    /// The move is part of @ref play, not a single @ref goTo.
    bool m_sequence = false;
    /// @ref play starts again after the last waypoint.
    bool m_loop = false;
    /// Index of the waypoint approached, or -1.
    int m_cursor = -1;
    /// Frame chosen by @ref frame; empty for the tool.
    std::string m_frame;
    /// Why the last command failed.
    std::string m_error;
};

} // namespace robotik
