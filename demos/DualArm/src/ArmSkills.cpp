// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "ArmSkills.hpp"

#include "RobotIK/Task/Perception/WorldModel.hpp"
#include "RobotIK/Robot/Robot.hpp"
#include "RobotIK/Task/RobotContext.hpp"

#include <cmath>
#include <numbers>

namespace dual_arm
{

using robotik::Status;

robotik::Vector3 inBase(robotik::RobotContext const& p_context,
                        robotik::Vector3 const& p_belief)
{
    if (p_context.team.empty())
    {
        return p_belief; // A robot alone: the beliefs are in its base frame.
    }
    return p_context.robot.getBase().pose.inverse() * p_belief;
}

robotik::Quaternion pointingDown(robotik::Vector3 const& p_at)
{
    // Half a turn about z from the yaw facing the point: the wrist of the
    // 6-axis arm is then near 0 instead of near its limit.
    double const yaw = std::atan2(p_at.y, p_at.x) + std::numbers::pi;
    return { 0.0, std::cos(yaw * 0.5), std::sin(yaw * 0.5), 0.0 };
}

robotik::Robot const* teammate(robotik::RobotContext const& p_context,
                               std::string_view p_id)
{
    if (p_context.robot.getId() == p_id)
    {
        return &p_context.robot;
    }
    for (robotik::Robot const* robot : p_context.team)
    {
        if (robot->getId() == p_id)
        {
            return robot;
        }
    }
    return nullptr;
}

robotik::VacuumGripper* cupOf(robotik::Robot& p_robot)
{
    return p_robot.getActuators().first<robotik::VacuumGripper>();
}

robotik::VacuumGripper const* cupOf(robotik::Robot const& p_robot)
{
    return p_robot.getActuators().first<robotik::VacuumGripper>();
}

std::string heldObject(robotik::Robot const& p_robot)
{
    robotik::VacuumGripper const* cup = cupOf(p_robot);
    return cup != nullptr && cup->holding() ? cup->label() : std::string{};
}

// ----------------------------------------------------------------------------
HoverSkill::HoverSkill(std::string p_object, robotik::Length p_clearance)
    : m_object(std::move(p_object)), m_clearance(p_clearance)
{
}

void HoverSkill::reset()
{
    m_planned = false;
    m_move.reset();
}

Status HoverSkill::tick(robotik::RobotContext& p_context, robotik::Seconds p_dt)
{
    if (!m_planned)
    {
        // Believed pose (perception or oracle), never the ground truth.
        robotik::WorldObject const* object =
            p_context.world_model.find(m_object);
        robotik::VacuumGripper const* cup = cupOf(p_context.robot);
        if (object == nullptr || cup == nullptr)
        {
            return object == nullptr ? fail("unknown_object",
                                            m_object + " is not in the beliefs")
                                     : fail("no_gripper", "no vacuum cup");
        }
        robotik::Vector3 const at = inBase(p_context, object->position);
        double const top =
            inBase(p_context, robotik::Vector3(0.0, 0.0, robotik::top(*object)))
                .z;
        double const height = top + m_clearance.value() + cup->length().value();
        robotik::Pose const target{ { at.x, at.y, height }, pointingDown(at) };
        m_move = robotik::MoveTCPSkill(cup->link(), target);
        m_planned = true;
    }
    Status const status = m_move.tick(p_context, p_dt);
    if (status == Status::FAILURE)
    {
        return fail("no_ik_solution", "no joint solution above " + m_object);
    }
    return status;
}

void HoverSkill::cancel(robotik::RobotContext& p_context)
{
    m_move.cancel(p_context);
}

// ----------------------------------------------------------------------------
GraspSkill::GraspSkill(std::string p_object, robotik::Seconds p_timeout)
    : m_object(std::move(p_object)), m_timeout(p_timeout)
{
}

Status GraspSkill::tick(robotik::RobotContext& p_context, robotik::Seconds p_dt)
{
    robotik::VacuumGripper* cup = cupOf(p_context.robot);
    if (cup == nullptr)
    {
        return fail("no_gripper", "no vacuum cup on the robot");
    }
    cup->suction(true);
    // The vacuum holds: the cup took what it was sent to take.
    if (cup->holding())
    {
        cup->label(m_object);
        return Status::SUCCESS;
    }
    m_waited += p_dt;
    if (m_waited > m_timeout)
    {
        cup->suction(false);
        return fail("grasp_not_confirmed", "suction holds nothing");
    }
    return Status::RUNNING;
}

void GraspSkill::cancel(robotik::RobotContext& p_context)
{
    if (robotik::VacuumGripper* cup = cupOf(p_context.robot);
        cup != nullptr && !cup->holding())
    {
        cup->suction(false);
    }
}

// ----------------------------------------------------------------------------
Status ReleaseSkill::tick(robotik::RobotContext& p_context,
                          robotik::Seconds p_dt)
{
    robotik::VacuumGripper* cup = cupOf(p_context.robot);
    if (cup == nullptr)
    {
        return fail("no_gripper", "no vacuum cup on the robot");
    }
    cup->suction(false);
    if (!cup->holding())
    {
        return Status::SUCCESS;
    }
    m_waited += p_dt;
    if (m_waited > m_timeout)
    {
        return fail("release_not_confirmed",
                    "the part is still held after the suction stopped");
    }
    return Status::RUNNING;
}

} // namespace dual_arm
