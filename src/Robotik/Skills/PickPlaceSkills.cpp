// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "Robotik/Skills/PickPlaceSkills.hpp"

#include "Robotik/ECS/ObjectComponents.hpp"
#include "Robotik/Perception/WorldModel.hpp"
#include "Robotik/Runtime/RobotContext.hpp"

#include "Compages/World/World.hpp"

#include <cmath>

namespace robotik
{

VacuumGripper* findGripper(Robot const& p_robot, std::string const& p_name)
{
    return p_name.empty() ? p_robot.actuators().first<VacuumGripper>()
                          : p_robot.actuators().find<VacuumGripper>(p_name);
}

ApproachSkill::ApproachSkill(std::string p_object,
                             Length p_clearance,
                             std::string p_gripper)
    : m_object(std::move(p_object)),
      m_clearance(p_clearance),
      m_gripper(std::move(p_gripper))
{
}

void ApproachSkill::reset()
{
    m_planned = false;
    m_move.reset();
}

Status ApproachSkill::tick(RobotContext& p_context, Seconds p_dt)
{
    if (!m_planned)
    {
        WorldObject const* object = p_context.world_model.find(m_object);
        VacuumGripper const* gripper = findGripper(p_context.robot, m_gripper);
        if (object == nullptr || gripper == nullptr)
        {
            return Status::FAILURE;
        }

        // Flange above the cup, cup pointing down: half a turn about X, then
        // facing the object from the base so the wrist stays mid range.
        Vector3 const at = object->position;
        double const yaw = std::atan2(at.y, at.x);
        Pose const target{
            { at.x,
              at.y,
              object->top() + (m_clearance + gripper->length()).value() },
            { 0.0, std::cos(yaw * 0.5), std::sin(yaw * 0.5), 0.0 }
        };
        m_move = MoveTCPSkill(gripper->link(), target);
        m_planned = true;
    }
    return m_move.tick(p_context, p_dt);
}

void ApproachSkill::cancel(RobotContext& p_context)
{
    m_move.cancel(p_context);
}

GraspSkill::GraspSkill(std::string p_object,
                       std::string p_gripper,
                       Seconds p_timeout)
    : m_object(std::move(p_object)),
      m_gripper(std::move(p_gripper)),
      m_timeout(p_timeout)
{
}

Status GraspSkill::tick(RobotContext& p_context, Seconds p_dt)
{
    VacuumGripper* gripper = findGripper(p_context.robot, m_gripper);
    if (gripper == nullptr)
    {
        return Status::FAILURE;
    }
    gripper->suction(true);
    if (gripper->holding())
    {
        compages::world::World& world = p_context.robot.world();
        auto const* held = world.alive(gripper->held())
                               ? world.entity(gripper->held())
                                     .find<ecs::SceneObject>()
                               : nullptr;
        if (held != nullptr && held->name == m_object)
        {
            return Status::SUCCESS;
        }
    }
    m_waited += p_dt;
    if (m_waited > m_timeout)
    {
        gripper->suction(false);
        return Status::FAILURE;
    }
    return Status::RUNNING;
}

void GraspSkill::cancel(RobotContext& p_context)
{
    if (VacuumGripper* gripper = findGripper(p_context.robot, m_gripper);
        gripper != nullptr && !gripper->holding())
    {
        gripper->suction(false);
    }
}

ReleaseSkill::ReleaseSkill(std::string p_gripper, Seconds p_timeout)
    : m_gripper(std::move(p_gripper)), m_timeout(p_timeout)
{
}

Status ReleaseSkill::tick(RobotContext& p_context, Seconds p_dt)
{
    VacuumGripper* gripper = findGripper(p_context.robot, m_gripper);
    if (gripper == nullptr)
    {
        return Status::FAILURE;
    }
    gripper->suction(false);
    if (!gripper->holding())
    {
        return Status::SUCCESS;
    }
    m_waited += p_dt;
    return m_waited > m_timeout ? Status::FAILURE : Status::RUNNING;
}

DetectSkill::DetectSkill(std::string p_object, Seconds p_timeout)
    : m_object(std::move(p_object)), m_timeout(p_timeout)
{
}

Status DetectSkill::tick(RobotContext& p_context, Seconds /*p_dt*/)
{
    if (!m_started)
    {
        m_start = p_context.time;
        m_started = true;
    }
    WorldObject const* object = p_context.world_model.find(m_object);
    if (object == nullptr)
    {
        return Status::FAILURE;
    }
    if (object->observed() && object->seen >= m_start)
    {
        return Status::SUCCESS;
    }
    return p_context.time - m_start > m_timeout ? Status::FAILURE
                                                : Status::RUNNING;
}

} // namespace robotik
