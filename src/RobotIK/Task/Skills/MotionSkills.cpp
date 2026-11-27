// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "RobotIK/Task/Skills/MotionSkills.hpp"

#include "RobotIK/Task/RobotContext.hpp"

#include <algorithm>
#include <cmath>

namespace robotik
{

static std::vector<JointId> jointsOf(Robot const& p_robot,
                                     std::string const& p_group)
{
    if (!p_group.empty())
    {
        if (auto const* group =
                p_robot.getActuators().find<JointGroup>(p_group))
        {
            return { group->joints().begin(), group->joints().end() };
        }
        return {};
    }
    std::vector<JointId> joints(p_robot.getJoints().size());
    for (std::size_t i = 0; i < joints.size(); ++i)
    {
        joints[i] = static_cast<JointId>(i);
    }
    return joints;
}

static void holdJoints(JointSet& p_joints, std::span<JointId const> p_ids)
{
    for (JointId const id : p_ids)
    {
        p_joints.hold(id);
    }
}

HomeSkill::HomeSkill(std::string p_group, double p_tolerance, double p_speed)
    : m_group(std::move(p_group)), m_tolerance(p_tolerance), m_speed(p_speed)
{
}

void HomeSkill::reset()
{
    m_bound = false;
}

Status HomeSkill::tick(RobotContext& p_context, Seconds /*p_dt*/)
{
    JointSet& joints = p_context.robot.getJoints();
    if (!m_bound)
    {
        m_joints = jointsOf(p_context.robot, m_group);
        m_bound = true;
    }
    if (m_joints.empty())
    {
        return fail("no_joints",
                    m_group.empty() ? std::string("the robot has no joint")
                                    : "the group " + m_group + " has no joint");
    }
    bool reached = true;
    for (JointId const id : m_joints)
    {
        joints.moveTo(id, joints.home(id));
        reached = reached && joints.reached(id, m_tolerance, m_speed);
    }
    return reached ? Status::SUCCESS : Status::RUNNING;
}

void HomeSkill::cancel(RobotContext& p_context)
{
    holdJoints(p_context.robot.getJoints(), m_joints);
}

MoveJointSkill::MoveJointSkill(std::string p_joint,
                               double p_target,
                               double p_tolerance,
                               double p_speed)
    : m_name(std::move(p_joint)),
      m_target(p_target),
      m_tolerance(p_tolerance),
      m_speed(p_speed)
{
}

void MoveJointSkill::reset()
{
    m_joint = NO_JOINT;
}

Status MoveJointSkill::tick(RobotContext& p_context, Seconds /*p_dt*/)
{
    JointSet& joints = p_context.robot.getJoints();
    if (m_joint == NO_JOINT)
    {
        m_joint = joints.find(m_name);
        if (m_joint == NO_JOINT)
        {
            return fail("unknown_joint", "no joint " + m_name);
        }
    }
    joints.moveTo(m_joint, m_target);
    return joints.reached(m_joint, m_tolerance, m_speed) ? Status::SUCCESS
                                                : Status::RUNNING;
}

void MoveJointSkill::cancel(RobotContext& p_context)
{
    if (m_joint != NO_JOINT)
    {
        p_context.robot.getJoints().hold(m_joint);
    }
}

MoveJointsSkill::MoveJointsSkill(JointPosture p_targets,
                                 double p_tolerance,
                                 double p_speed)
    : m_targets(std::move(p_targets)),
      m_tolerance(p_tolerance),
      m_speed(p_speed)
{
}

void MoveJointsSkill::reset()
{
    m_joints.clear();
    m_goals.clear();
}

Status MoveJointsSkill::tick(RobotContext& p_context, Seconds /*p_dt*/)
{
    JointSet& joints = p_context.robot.getJoints();
    if (m_joints.empty())
    {
        for (auto const& [name, goal] : m_targets)
        {
            JointId const id = joints.find(name);
            if (id == NO_JOINT)
            {
                return fail("unknown_joint", "no joint " + name);
            }
            m_joints.push_back(id);
            m_goals.push_back(goal);
        }
        if (m_joints.empty())
        {
            return fail("no_joints", "no joint to move");
        }
    }
    bool reached = true;
    for (std::size_t i = 0; i < m_joints.size(); ++i)
    {
        joints.moveTo(m_joints[i], m_goals[i]);
        reached = reached && joints.reached(m_joints[i], m_tolerance, m_speed);
    }
    return reached ? Status::SUCCESS : Status::RUNNING;
}

void MoveJointsSkill::cancel(RobotContext& p_context)
{
    holdJoints(p_context.robot.getJoints(), m_joints);
}

MoveTCPSkill::MoveTCPSkill(std::string p_frame,
                           Pose const& p_target,
                           double p_tolerance,
                           double p_speed)
    : m_frame(std::move(p_frame)),
      m_target(p_target),
      m_tolerance(p_tolerance),
      m_speed(p_speed)
{
}

void MoveTCPSkill::goal(Pose const& p_target)
{
    m_target = p_target;
    m_planned = false;
}

void MoveTCPSkill::reset()
{
    m_planned = false;
}

Status MoveTCPSkill::tick(RobotContext& p_context, Seconds /*p_dt*/)
{
    Robot& robot = p_context.robot;
    if (!m_planned)
    {
        std::string const& frame = m_frame.empty() ? robot.getTool() : m_frame;
        auto solution = robot.inverseKinematics(frame, m_target);
        if (!solution)
        {
            return fail("no_ik_solution",
                        "no joint solution brings " + frame + " to the target");
        }
        m_solution = std::move(*solution);
        m_planned = true;
    }

    JointSet& joints = robot.getJoints();
    bool reached = true;
    for (JointId id = 0; id < joints.size(); ++id)
    {
        joints.moveTo(id, m_solution[id]);
        reached = reached && joints.reached(id, m_tolerance, m_speed);
    }
    return reached ? Status::SUCCESS : Status::RUNNING;
}

void MoveTCPSkill::cancel(RobotContext& p_context)
{
    p_context.robot.getJoints().hold();
}

Status StopSkill::tick(RobotContext& p_context, Seconds /*p_dt*/)
{
    if (!m_held)
    {
        p_context.robot.getJoints().hold();
        m_held = true;
    }
    return Status::RUNNING;
}

} // namespace robotik
