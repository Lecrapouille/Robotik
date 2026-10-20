// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file MotionSkills.hpp
/// Joint-space and Cartesian motions, and the emergency stop.
///
/// Joint ids are resolved once at @ref robotik::Skill::reset, so ticks only
/// touch the dense @ref robotik::JointSet arrays. A cancelled motion holds the
/// joints where they are.
///
/// @par Contract
/// - Promises: a cancelled motion holds the joints where they are.
/// - Expects: joint names exist in the robot.
#pragma once

#include "Robotik/Common/Math.hpp"
#include "Robotik/Robot/Joints.hpp"
#include "Robotik/Robot/Robot.hpp"
#include "Robotik/Task/Skills/Skill.hpp"

#include <string>
#include <vector>

namespace robotik
{

// ****************************************************************************
/// Moves the joints of a group (all joints by default) to their home.
///
/// The home of a joint is set by the world file (@c home:) or by
/// @ref JointSet::home.
///
/// @code
/// scheduler.add<robotik::HomeSkill>(
///     { .name = "Home", .resources = { resources.require("arm") } }, "arm");
/// @endcode
// ****************************************************************************
class HomeSkill final: public Skill
{
public:

    // -------------------------------------------------------------------------
    /// @param p_group @ref JointGroup name; empty for every joint.
    /// @param p_tolerance Largest error (rad or m) to call a joint reached.
    // -------------------------------------------------------------------------
    explicit HomeSkill(std::string p_group = {}, double p_tolerance = 1e-2);

    /// Forgets the joints, resolved again at the next tick.
    void reset() override;
    /// Commands the home positions; succeeds when every joint is there.
    Status tick(RobotContext& p_context, Seconds p_dt) override;
    /// Holds the joints where they are.
    void cancel(RobotContext& p_context) override;

private:

    /// Joint group name; empty for every joint.
    std::string m_group;
    /// Largest error to call a joint reached.
    double m_tolerance;
    /// Joints of the group.
    std::vector<JointId> m_joints;
    /// @ref m_joints is resolved.
    bool m_bound = false;
};

// ****************************************************************************
/// Drives one joint to a position.
///
/// @code
/// auto const id = scheduler.add<robotik::MoveJointSkill>(
///     { .name = "Raise" }, "joint2", 0.5);
/// static_cast<robotik::MoveJointSkill&>(scheduler.skill(id)).goal(0.8);
/// scheduler.request(id);
/// @endcode
// ****************************************************************************
class MoveJointSkill final: public Skill
{
public:

    // -------------------------------------------------------------------------
    /// @param p_joint Joint name.
    /// @param p_target Target position (rad or m).
    /// @param p_tolerance Largest error to call the target reached.
    // -------------------------------------------------------------------------
    MoveJointSkill(std::string p_joint,
                   double p_target,
                   double p_tolerance = 1e-2);

    // -------------------------------------------------------------------------
    /// Changes the target, used from the next run.
    /// @param p_target Target position (rad or m).
    // -------------------------------------------------------------------------
    void goal(double p_target)
    {
        m_target = p_target;
    }

    /// Finds the joint by name.
    void reset() override;
    /// Commands the target; succeeds once it is reached.
    Status tick(RobotContext& p_context, Seconds p_dt) override;
    /// Holds the joint where it is.
    void cancel(RobotContext& p_context) override;

private:

    /// Joint name.
    std::string m_name;
    /// Target position.
    double m_target;
    /// Largest error to call the target reached.
    double m_tolerance;
    /// Joint resolved at reset.
    JointId m_joint = NO_JOINT;
};

// ****************************************************************************
/// Drives several joints at once; succeeds when all are reached.
///
/// @code
/// scheduler.add<robotik::MoveJointsSkill>(
///     { .name = "Ready" }, robotik::JointPosture{ { "joint1", 0.0 }, {
///     "joint2", 0.6 } });
/// @endcode
// ****************************************************************************
class MoveJointsSkill final: public Skill
{
public:

    // -------------------------------------------------------------------------
    /// @param p_targets Joint name to target position (rad or m).
    /// @param p_tolerance Largest error to call a joint reached.
    // -------------------------------------------------------------------------
    explicit MoveJointsSkill(JointPosture p_targets, double p_tolerance = 1e-2);

    /// Finds the joints by name.
    void reset() override;
    /// Commands the targets; succeeds when every joint is reached.
    Status tick(RobotContext& p_context, Seconds p_dt) override;
    /// Holds the joints where they are.
    void cancel(RobotContext& p_context) override;

private:

    /// Joint name to target position.
    JointPosture m_targets;
    /// Largest error to call a joint reached.
    double m_tolerance;
    /// Joints resolved at reset.
    std::vector<JointId> m_joints;
    /// Target of each joint of @ref m_joints.
    std::vector<double> m_goals;
};

// ****************************************************************************
/// Moves a frame to a pose in the robot base frame (IK once per goal).
///
/// The motion is in joint space: the frame reaches the pose, but does not
/// follow a straight line.
///
/// @code
/// robotik::Pose target;
/// target.position = robotik::Vector3(0.4, 0.0, 0.3);
/// target.rotation = robotik::Quaternion::fromRpy(Radians(3.1416), Radians(0.0), Radians(0.0));
/// scheduler.add<robotik::MoveTCPSkill>({ .name = "Above" }, "", target);
/// @endcode
// ****************************************************************************
class MoveTCPSkill final: public Skill
{
public:

    // -------------------------------------------------------------------------
    /// @param p_frame Link to move; empty for the robot tool frame.
    /// @param p_target Pose of the frame in the robot base frame.
    /// @param p_tolerance Largest joint error to call the pose reached.
    // -------------------------------------------------------------------------
    explicit MoveTCPSkill(std::string p_frame = {},
                          Pose const& p_target = {},
                          double p_tolerance = 1e-2);

    // -------------------------------------------------------------------------
    /// Sets a new goal, planned at the next tick.
    /// @param p_target Pose of the frame in the robot base frame.
    // -------------------------------------------------------------------------
    void goal(Pose const& p_target);

    /// Asks for a new IK solution.
    void reset() override;
    /// Solves IK once, then drives the joints; fails if IK does not converge.
    Status tick(RobotContext& p_context, Seconds p_dt) override;
    /// Holds the joints where they are.
    void cancel(RobotContext& p_context) override;

private:

    /// Link to move; empty for the tool frame.
    std::string m_frame;
    /// Goal pose in the robot base frame.
    Pose m_target;
    /// Largest joint error to call the pose reached.
    double m_tolerance;
    /// IK solution, indexed by @ref JointId.
    std::vector<double> m_solution;
    /// @ref m_solution matches @ref m_target.
    bool m_planned = false;
};

// ****************************************************************************
/// Holds every joint until cancelled. Give it the highest priority and all
/// motion resources: it preempts whatever moves.
///
/// @code
/// scheduler.add<robotik::StopSkill>(
///     { .name = "Stop", .resources = { resources.require("arm") },
///       .priority = 1000, .cancellable = false });
/// @endcode
// ****************************************************************************
class StopSkill final: public Skill
{
public:

    /// Holds again at the next tick.
    void reset() override
    {
        m_held = false;
    }

    /// Holds every joint once, then keeps running.
    Status tick(RobotContext& p_context, Seconds p_dt) override;

private:

    /// The joints are held.
    bool m_held = false;
};

} // namespace robotik
