// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

//! @file MotionSkills.hpp
//! @brief Joint-space and Cartesian motions, and the emergency stop.
//!
//! Joint ids are resolved once at @ref Skill::reset, so ticks only touch the
//! dense @ref JointSet arrays. A cancelled motion holds the joints where they
//! are.
#pragma once

#include "Robotik/Math/Pose.hpp"
#include "Robotik/Robot/Joints.hpp"
#include "Robotik/Robot/Robot.hpp"
#include "Robotik/Skills/Skill.hpp"

#include <string>
#include <vector>

namespace robotik
{

// ****************************************************************************
//! @brief Moves the joints of a group (all joints by default) to their home.
// ****************************************************************************
class HomeSkill final: public Skill
{
public:

    //! @param p_group @ref JointGroup name; empty for every joint.
    explicit HomeSkill(std::string p_group = {}, double p_tolerance = 1e-2);

    void reset() override;
    Status tick(RobotContext& p_context, Seconds p_dt) override;
    void cancel(RobotContext& p_context) override;

private:

    std::string m_group;
    double m_tolerance;
    std::vector<JointId> m_joints;
    bool m_bound = false;
};

// ****************************************************************************
//! @brief Drives one joint to a position.
// ****************************************************************************
class MoveJointSkill final: public Skill
{
public:

    MoveJointSkill(std::string p_joint, double p_target, double p_tolerance = 1e-2);

    void goal(double p_target)
    {
        m_target = p_target;
    }

    void reset() override;
    Status tick(RobotContext& p_context, Seconds p_dt) override;
    void cancel(RobotContext& p_context) override;

private:

    std::string m_name;
    double m_target;
    double m_tolerance;
    JointId m_joint = NO_JOINT;
};

// ****************************************************************************
//! @brief Drives several joints at once; succeeds when all are reached.
// ****************************************************************************
class MoveJointsSkill final: public Skill
{
public:

    explicit MoveJointsSkill(JointPosture p_targets, double p_tolerance = 1e-2);

    void reset() override;
    Status tick(RobotContext& p_context, Seconds p_dt) override;
    void cancel(RobotContext& p_context) override;

private:

    JointPosture m_targets;
    double m_tolerance;
    std::vector<JointId> m_joints;
    std::vector<double> m_goals;
};

// ****************************************************************************
//! @brief Moves a frame to a pose in the robot base frame (IK once per goal).
// ****************************************************************************
class MoveTCPSkill final: public Skill
{
public:

    //! @param p_frame Link to move; empty for the robot tool frame.
    explicit MoveTCPSkill(std::string p_frame = {},
                          Pose const& p_target = {},
                          double p_tolerance = 1e-2);

    //! @brief New goal, planned at the next tick.
    void goal(Pose const& p_target);

    void reset() override;
    Status tick(RobotContext& p_context, Seconds p_dt) override;
    void cancel(RobotContext& p_context) override;

private:

    std::string m_frame;
    Pose m_target;
    double m_tolerance;
    std::vector<double> m_solution;
    bool m_planned = false;
};

// ****************************************************************************
//! @brief Holds every joint until cancelled. Give it the highest priority and
//! all motion resources: it preempts whatever moves.
// ****************************************************************************
class StopSkill final: public Skill
{
public:

    void reset() override
    {
        m_held = false;
    }

    Status tick(RobotContext& p_context, Seconds p_dt) override;

private:

    bool m_held = false;
};

} // namespace robotik
