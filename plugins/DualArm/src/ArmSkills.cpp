// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "ArmSkills.hpp"

#include "Robotik/ECS/ObjectComponents.hpp"
#include "Robotik/Perception/WorldModel.hpp"
#include "Robotik/Robot/Robot.hpp"
#include "Robotik/Runtime/RobotContext.hpp"

#include "Compages/World/World.hpp"

#include <cmath>
#include <numbers>

namespace dual_arm
{

using robotik::Status;

robotik::Vector3 inBase(robotik::RobotContext const& p_context, robotik::Vector3 const& p_belief)
{
    if (p_context.team.empty())
    {
        return p_belief; // A robot alone: the beliefs are in its base frame.
    }
    return p_context.robot.base().pose.inverse() * p_belief;
}

robotik::Quaternion pointingDown(robotik::Vector3 const& p_at)
{
    // Half a turn about z from the yaw facing the point: the wrist of the
    // 6-axis arm is then near 0 instead of near its limit.
    double const yaw = std::atan2(p_at.y, p_at.x) + std::numbers::pi;
    return { 0.0, std::cos(yaw * 0.5), std::sin(yaw * 0.5), 0.0 };
}

robotik::Robot const* teammate(robotik::RobotContext const& p_context, std::string_view p_id)
{
    if (p_context.robot.id() == p_id)
    {
        return &p_context.robot;
    }
    for (robotik::Robot const* robot : p_context.team)
    {
        if (robot->id() == p_id)
        {
            return robot;
        }
    }
    return nullptr;
}

robotik::VacuumGripper* cupOf(robotik::Robot const& p_robot)
{
    return p_robot.actuators().first<robotik::VacuumGripper>();
}

std::string heldObject(robotik::Robot const& p_robot)
{
    robotik::VacuumGripper const* cup = cupOf(p_robot);
    if (cup == nullptr || !cup->holding())
    {
        return {};
    }
    compages::world::World& world = p_robot.world();
    auto const* held = world.alive(cup->held())
                           ? world.entity(cup->held()).find<robotik::ecs::Shape>()
                           : nullptr;
    return held != nullptr ? held->name : std::string{};
}

// ----------------------------------------------------------------------------
HoverSkill::HoverSkill(std::string p_object, Length p_clearance)
    : m_object(std::move(p_object)), m_clearance(p_clearance)
{
}

void HoverSkill::reset()
{
    m_planned = false;
    m_why.clear();
    m_move.reset();
}

Status HoverSkill::tick(robotik::RobotContext& p_context, Seconds p_dt)
{
    if (!m_planned)
    {
        // Believed pose (perception or oracle), never the ground truth.
        robotik::WorldObject const* object = p_context.world_model.find(m_object);
        robotik::VacuumGripper const* cup = cupOf(p_context.robot);
        if (object == nullptr || cup == nullptr)
        {
            m_why = object == nullptr ? m_object + " is not in the beliefs" : "no vacuum cup";
            return Status::FAILURE;
        }
        robotik::Vector3 const at = inBase(p_context, object->position);
        double const top = inBase(p_context, robotik::Vector3(0.0, 0.0, object->top())).z;
        double const height = top + m_clearance.value() + cup->length().value();
        robotik::Pose const target{ { at.x, at.y, height }, pointingDown(at) };
        m_move = robotik::MoveTCPSkill(cup->link(), target);
        m_planned = true;
    }
    Status const status = m_move.tick(p_context, p_dt);
    if (status == Status::FAILURE)
    {
        m_why = "no joint solution above " + m_object;
    }
    return status;
}

void HoverSkill::cancel(robotik::RobotContext& p_context)
{
    m_move.cancel(p_context);
}

// ----------------------------------------------------------------------------
GraspSkill::GraspSkill(std::string p_object, Seconds p_timeout)
    : m_object(std::move(p_object)), m_timeout(p_timeout)
{
}

Status GraspSkill::tick(robotik::RobotContext& p_context, Seconds p_dt)
{
    robotik::VacuumGripper* cup = cupOf(p_context.robot);
    if (cup == nullptr)
    {
        m_why = "no vacuum cup on the robot";
        return Status::FAILURE;
    }
    cup->suction(true);
    if (heldObject(p_context.robot) == m_object)
    {
        return Status::SUCCESS;
    }
    m_waited += p_dt;
    if (m_waited > m_timeout)
    {
        cup->suction(false);
        std::string const held = heldObject(p_context.robot);
        m_why = held.empty() ? "suction holds nothing" : "suction holds " + held + ", not " + m_object;
        return Status::FAILURE;
    }
    return Status::RUNNING;
}

void GraspSkill::cancel(robotik::RobotContext& p_context)
{
    if (robotik::VacuumGripper* cup = cupOf(p_context.robot); cup != nullptr && !cup->holding())
    {
        cup->suction(false);
    }
}

// ----------------------------------------------------------------------------
Status ReleaseSkill::tick(robotik::RobotContext& p_context, Seconds p_dt)
{
    robotik::VacuumGripper* cup = cupOf(p_context.robot);
    if (cup == nullptr)
    {
        m_why = "no vacuum cup on the robot";
        return Status::FAILURE;
    }
    cup->suction(false);
    if (!cup->holding())
    {
        return Status::SUCCESS;
    }
    m_waited += p_dt;
    if (m_waited > m_timeout)
    {
        m_why = "the part is still held after the suction stopped";
        return Status::FAILURE;
    }
    return Status::RUNNING;
}

} // namespace dual_arm
