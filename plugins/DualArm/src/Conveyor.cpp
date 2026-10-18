// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "Conveyor.hpp"

#include "Robotik/Perception/WorldModel.hpp"
#include "Robotik/Runtime/RobotContext.hpp"
#include "Robotik/Runtime/Simulation.hpp"

#include "Compages/World/World.hpp"

#include <cmath>

namespace dual_arm
{

/// Belt speed (m/s).
static constexpr double kSpeed = 0.25;

void Conveyor::carry(std::string p_part, robotik::Vector3 const& p_goal)
{
    m_part = std::move(p_part);
    m_goal = p_goal;
    m_delivered.clear();
    m_running = true;
}

void Conveyor::stop()
{
    m_running = false;
}

void Conveyor::reset()
{
    m_part.clear();
    m_delivered.clear();
    m_running = false;
}

void Conveyor::step(robotik::Simulation& p_simulation, Seconds p_dt)
{
    if (!m_running || !p_simulation.robot().resources().available(RESOURCE))
    {
        return;
    }
    compages::world::Entity part = p_simulation.entity(m_part);
    if (!part)
    {
        m_running = false;
        return;
    }
    // Child of the Z-up frame of the world: its local position is the world one.
    auto at = part.position();
    double const dx = m_goal.x - at.x;
    double const dy = m_goal.y - at.y;
    double const left = std::hypot(dx, dy);
    double const travel = kSpeed * p_dt.value();
    if (left <= travel)
    {
        at.x = static_cast<float>(m_goal.x);
        at.y = static_cast<float>(m_goal.y);
        m_delivered = m_part;
        m_running = false;
    }
    else
    {
        at.x += static_cast<float>(travel * dx / left);
        at.y += static_cast<float>(travel * dy / left);
    }
    part.position(at);
}

// ----------------------------------------------------------------------------
ConveySkill::ConveySkill(Conveyor& p_conveyor, std::string p_part, std::string p_zone)
    : m_conveyor(p_conveyor), m_part(std::move(p_part)), m_zone(std::move(p_zone))
{
}

robotik::Status ConveySkill::tick(robotik::RobotContext& p_context, Seconds /*p_dt*/)
{
    if (!m_started)
    {
        // The belt runs in the world frame, as the shared beliefs.
        robotik::WorldObject const* zone = p_context.world_model.find(m_zone);
        if (zone == nullptr)
        {
            m_why = m_zone + " is not in the beliefs";
            return robotik::Status::FAILURE;
        }
        m_conveyor.carry(m_part, zone->position);
        m_started = true;
    }
    return m_conveyor.delivered() == m_part ? robotik::Status::SUCCESS : robotik::Status::RUNNING;
}

void ConveySkill::cancel(robotik::RobotContext& /*p_context*/)
{
    m_conveyor.stop();
}

} // namespace dual_arm
