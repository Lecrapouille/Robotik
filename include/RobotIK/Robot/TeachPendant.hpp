// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file TeachPendant.hpp
/// A virtual teach pendant: the hand-held box that an operator uses to move
/// a robot by small steps, save the positions that the robot must go to, and
/// replay them. The page doc/concepts/07-teach-pendant.md explains the words
/// (jog, waypoint, smooth move) for a reader who never used a robot.
///
/// The pendant gives the motors the joint positions to reach (the
/// targets). It does not move the arm itself: the arm moves when the
/// simulation (or the real robot) steps.
///
/// @par Contract
/// - Promises: the pendant changes only the joint targets of the robot; it
///   never steps the physics and never changes the robot model.
/// - Expects: a @ref robotik::RobotRuntime steps after each command, and
///   nothing else (a mission, a behavior tree) writes targets at the same
///   time.
/// - Errors: a command that cannot be done returns @c false, keeps the
///   targets as they were and tells why in @ref TeachPendant::getError.
#pragma once

#include "RobotIK/Common/Math.hpp"
#include "RobotIK/Robot/Joints.hpp"
#include "RobotIK/Robot/Robot.hpp"

#include <array>
#include <cstddef>
#include <span>
#include <string>
#include <utility>
#include <vector>

namespace robotik
{

// ****************************************************************************
/// Teach a robot a motion by hand, then replay it.
///
/// Why: on a factory floor, an operator does not write code to give a path
/// to an arm. They push the arm step by step with the buttons of a teach
/// pendant, save the good positions, and the robot replays them. This class
/// gives the same tool in the simulator, so that you can make a motion
/// before you write a skill, or check that a robot can reach a place.
///
/// When: use it for manual work (the Teach Pendant panel of the simulator,
/// a quick test). A mission uses skills and trajectories, not this class.
///
/// How: there are three kinds of command.
/// - Jog: move by a small step. @ref jogJoint turns (or slides) one joint;
///   @ref jogTool moves the tool in a straight line or turns it, and finds
///   the joint angles that put it there (inverse kinematics, see
///   doc/reference/glossary.md).
/// - Record: @ref record saves where every joint is now, as a
///   @ref Waypoint.
/// - Replay: @ref goTo goes to one waypoint, @ref play goes to all of them
///   in order. The arm starts and stops without a jerk. Call @ref update at
///   each step of the simulation while @ref isPlaying is true: it gives the
///   motors the next targets of the move.
///
/// Limits: between two waypoints each joint moves on its own, so the tool
/// does not follow a straight line. Pause the mission of the scenario before
/// you jog: if not, the mission writes its own targets at the next step and
/// the arm does not obey you.
///
/// @code
/// using namespace robotik::literals; // 0.01_m, 0.2_rad, 2.0_s
/// robotik::TeachPendant pendant;
/// // The operator lifts the tool by 1 cm and saves this position.
/// pendant.jogTool(robot, { 0.0_m, 0.0_m, 0.01_m },
///                 robotik::Vector3rad::ZERO);
/// pendant.record(robot, "above", 2.0_s); // reached in 2 s during replay
/// // They turn the last joint (the wrist) by 0.2 rad and save it too.
/// pendant.jogJoint(robot, wrist, 0.2_rad);
/// pendant.record(robot, "turned", 1.0_s);
/// // The robot replays the two positions, again and again.
/// pendant.play(robot, true);
/// while (pendant.isPlaying()) {
///     pendant.update(robot, dt);   // next targets of the move
///     runtime.step(dt);            // the physics moves the arm
/// }
/// @endcode
// ****************************************************************************
class TeachPendant
{
public:

    // ************************************************************************
    /// A position of the robot saved by the operator, so that the robot can
    /// come back exactly there.
    ///
    /// It keeps the value of every joint, not only the place of the tool:
    /// an arm can put its tool at the same place with different shapes
    /// (elbow up, elbow down). The joint values say which shape.
    ///
    /// @code
    /// robotik::TeachPendant::Waypoint const& w = pendant.getWaypoints()[0];
    /// std::cout << w.label << " is reached in " << w.duration.value() << " s\n";
    /// @endcode
    // ************************************************************************
    struct Waypoint
    {
        /// Name shown in the list of the panel ("above", "wp 2").
        std::string label;
        /// Value of each joint, in the order of @ref JointId: an angle in
        /// radians for a joint that turns, a length in metres for a joint
        /// that slides.
        std::vector<double> joints;
        /// Name of each joint of @ref joints, in the same order (the
        /// @ref Robot::Model::joints of the robot that recorded it). A replay
        /// on a robot whose joints differ is refused, even with the same
        /// count of joints.
        std::vector<std::string> joint_names;
        /// Where the moved frame (see @ref setFrame) was, in the frame of
        /// the robot base. Shown to the operator; replay uses @ref joints.
        Pose pose{};
        /// Time that the replay takes to come here from the waypoint before.
        /// A longer time gives a slower, softer move.
        Seconds duration{ 2.0 };
    };

    // -------------------------------------------------------------------------
    /// Turns (or slides) one joint by a small step, like the + and - keys of
    /// a real pendant. The value stays inside the limits of the joint. Stops
    /// a replay.
    /// @param p_robot Robot to move.
    /// @param p_joint Joint that turns (`joints.requireRevolute("joint6")`).
    /// @param p_delta Turn; 0.01 to 0.1 rad is usual. Negative goes the
    ///   other way.
    /// @return False when @p_joint is not a joint of @p_robot.
    // -------------------------------------------------------------------------
    bool jogJoint(Robot& p_robot, Revolute p_joint, Radians p_delta);

    // -------------------------------------------------------------------------
    /// Slides one joint (a linear axis, a gripper finger) by a small step;
    /// the same rules as the turning version above.
    /// @param p_robot Robot to move.
    /// @param p_joint Joint that slides (`joints.requirePrismatic("x")`).
    /// @param p_delta Step; 1 mm to 5 cm is usual. Negative goes back.
    /// @return False when @p_joint is not a joint of @p_robot.
    // -------------------------------------------------------------------------
    bool jogJoint(Robot& p_robot, Prismatic p_joint, Length p_delta);

    // -------------------------------------------------------------------------
    /// Moves or turns the chosen link (the tool by default) by a small step,
    /// along the axes of the robot base, and finds the joint values that put
    /// it there (see @ref getJogGoal). Stops a replay.
    /// @param p_robot Robot to move.
    /// @param p_translation Step along X, Y, Z; 1 mm to 5 cm is usual.
    /// @param p_rotation Turn about X, Y, Z; 0.01 to 0.3 rad is usual.
    /// @return False when there is no link to move, when its joints cannot
    ///   go in this direction, or when the place is out of reach.
    // -------------------------------------------------------------------------
    bool jogTool(Robot& p_robot,
                 Vector3m const& p_translation,
                 Vector3rad const& p_rotation);

    // -------------------------------------------------------------------------
    /// Saves where the robot is now, at the end of the list. It saves the
    /// measured joints (where the arm really is), not the targets: wait
    /// until the arm has stopped after a jog.
    /// @param p_robot Robot to read.
    /// @param p_label Name shown to the operator; empty gives "wp N".
    /// @param p_duration Time the replay takes to come to this waypoint;
    ///   less than 0.05 s becomes 0.05 s.
    /// @return The index of the new waypoint, for @ref goTo.
    // -------------------------------------------------------------------------
    std::size_t
    record(Robot const& p_robot, std::string p_label, Seconds p_duration);

    // -------------------------------------------------------------------------
    /// Starts the replay of every waypoint, first to last, each move with
    /// the duration of its waypoint. Call @ref update until @ref isPlaying is
    /// false (never, with @p_loop, until @ref stop).
    /// @param p_robot Robot to move.
    /// @param p_loop True to go back to the first waypoint after the last.
    /// @return False when the list is empty, or when one waypoint was saved
    ///   on a robot with other joints (see @ref Waypoint::joint_names).
    // -------------------------------------------------------------------------
    bool play(Robot& p_robot, bool p_loop);

    // -------------------------------------------------------------------------
    /// Starts a move to one waypoint, from the targets that the motors have
    /// now. The move takes the @ref Waypoint::duration of the waypoint and
    /// starts and ends at zero speed. Call @ref update until
    /// @ref isPlaying is false.
    /// @param p_robot Robot to move.
    /// @param p_index Index of the waypoint.
    /// @return False when there is no such waypoint, or when it was saved on
    ///   a robot with other joints (see @ref Waypoint::joint_names).
    // -------------------------------------------------------------------------
    bool goTo(Robot& p_robot, std::size_t p_index);

    // -------------------------------------------------------------------------
    /// Gives the motors the targets of the replay for this step. Without
    /// this call the arm does not move: the pendant has no clock of its own.
    /// Does nothing when @ref isPlaying is false.
    /// @param p_robot Robot to move.
    /// @param p_dt Time of the step, the same as the simulation step.
    // -------------------------------------------------------------------------
    void update(Robot& p_robot, Seconds p_dt);

    // -------------------------------------------------------------------------
    /// Stops a replay at once: the motors keep the arm where it is now
    /// (they do not finish the move).
    /// @param p_robot Robot to stop.
    // -------------------------------------------------------------------------
    void stop(Robot& p_robot);

    // -------------------------------------------------------------------------
    /// Removes one waypoint; the next ones move up by one. Stops a replay,
    /// because the indices change.
    /// @param p_index Index of the waypoint; out of range does nothing.
    // -------------------------------------------------------------------------
    void erase(std::size_t p_index);

    /// Removes every waypoint and stops a replay.
    void clear();

    // -------------------------------------------------------------------------
    /// Chooses the part of the robot that @ref jogTool moves and whose place
    /// @ref record saves. By default it is the tool, but it can be any link:
    /// the foot of a walking robot, the elbow of an arm.
    /// @param p_frame Name of a link of the robot. Empty: go back to the
    ///   tool (@ref Robot::getTool).
    // -------------------------------------------------------------------------
    void setFrame(std::string p_frame)
    {
        m_frame = std::move(p_frame);
    }

    // -------------------------------------------------------------------------
    /// @param p_robot Robot driven by the pendant (it gives the tool name).
    /// @return The name of the link moved by @ref jogTool; empty when no
    ///   link was chosen and the robot has no tool.
    // -------------------------------------------------------------------------
    [[nodiscard]] std::string const& getFrame(Robot const& p_robot) const;

    // -------------------------------------------------------------------------
    /// Tells in which directions @ref jogTool can move the chosen link now,
    /// so that a panel greys out the buttons that cannot work. The base of
    /// an arm turns about Z only; a link with no joint above it cannot move.
    /// @param p_robot Robot driven by the pendant.
    /// @return Six flags: move along X, Y, Z of the robot base, then turn
    ///   about X, Y, Z. All false when there is no link to move.
    // -------------------------------------------------------------------------
    [[nodiscard]] std::array<bool, 6> getJogAxes(Robot const& p_robot) const;

    // -------------------------------------------------------------------------
    /// Tells what @ref jogTool asks the inverse kinematics to keep. A link
    /// with six joints or more above it (the tool of an arm) can keep both
    /// its place and its orientation, so a straight move does not turn it.
    /// A link with fewer joints (an elbow, a knee) cannot: only what the jog
    /// changes is asked, the place for a move, the orientation for a turn.
    /// @param p_robot Robot driven by the pendant.
    /// @param p_rotation True for a turn, false for a straight move.
    /// @return What the inverse kinematics must reach.
    // -------------------------------------------------------------------------
    [[nodiscard]] IkGoal getJogGoal(Robot const& p_robot,
                                    bool p_rotation) const;

    // -------------------------------------------------------------------------
    /// @return The saved positions, in the order of replay.
    // -------------------------------------------------------------------------
    [[nodiscard]] std::span<Waypoint const> getWaypoints() const
    {
        return m_waypoints;
    }

    // -------------------------------------------------------------------------
    /// @return True while the arm goes to a waypoint (@ref goTo, @ref play).
    ///   A jog or @ref stop makes it false.
    // -------------------------------------------------------------------------
    [[nodiscard]] bool isPlaying() const
    {
        return m_playing;
    }

    // -------------------------------------------------------------------------
    /// @return The index of the waypoint the arm goes to now, to highlight
    ///   it in a list; -1 when nothing plays.
    // -------------------------------------------------------------------------
    [[nodiscard]] int getTargetIndex() const
    {
        return m_playing ? m_cursor : -1;
    }

    // -------------------------------------------------------------------------
    /// @return A sentence for the operator that says why the last command
    ///   returned false; empty after a command that worked.
    // -------------------------------------------------------------------------
    [[nodiscard]] std::string const& getError() const
    {
        return m_error;
    }

private:

    // -------------------------------------------------------------------------
    /// The smooth move of one joint: a polynomial of degree 5 (a "quintic")
    /// a0 + a3 t³ + a4 t⁴ + a5 t⁵. With these terms the speed and the
    /// acceleration are zero at the start and at the end, so the arm starts
    /// and stops without a jerk.
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

    /// Computes the smooth move of each joint toward @p_goal.
    void begin(Robot const& p_robot, Waypoint const& p_goal);
    /// Writes joint targets.
    void apply(Robot& p_robot, std::vector<double> const& p_positions);
    /// Current joint targets of the robot.
    [[nodiscard]] std::vector<double> command(Robot const& p_robot) const;
    /// True when @p_waypoint was recorded on the joints of @p_robot: same
    /// names, same order (see @ref Waypoint::joint_names).
    [[nodiscard]] bool compatible(Robot const& p_robot,
                                  Waypoint const& p_waypoint) const;

    /// Recorded waypoints.
    std::vector<Waypoint> m_waypoints;
    /// Smooth move of each joint for the active move.
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
    /// Link chosen by @ref setFrame; empty for the tool.
    std::string m_frame;
    /// Why the last command failed.
    std::string m_error;
};

} // namespace robotik
