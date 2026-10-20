// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "Conveyor.hpp"

#include "Robotik/Task/Perception/WorldModel.hpp"
#include "Robotik/Task/RobotContext.hpp"
#include "Robotik/Task/Cell.hpp"

#include "Compages/World/World.hpp"

#include <cmath>

namespace dual_arm
{

/// Belt speed (m/s).
static constexpr double SPEED = 0.40;
/// The part crosses to its lane this far in front of the feeder, where the
/// belt has no divider (m).
static constexpr double TRANSFER = 0.11;

void Conveyor::carry(std::string p_part, std::vector<robotik::Vector3> p_path)
{
    m_part = std::move(p_part);
    m_path = std::move(p_path);
    m_delivered.clear();
    m_running = !m_path.empty();
}

void Conveyor::stop()
{
    m_running = false;
}

void Conveyor::reset()
{
    m_part.clear();
    m_delivered.clear();
    m_path.clear();
    m_running = false;
}

void Conveyor::step(robotik::Cell& p_cell, robotik::Seconds p_dt)
{
    if (!m_running || !p_cell.robot().resources().available(RESOURCE))
    {
        return;
    }
    compages::world::Entity part = p_cell.entity(m_part);
    if (!part)
    {
        m_running = false;
        return;
    }
    // Child of the Z-up frame of the world: its local position is the world
    // one. The belt keeps its speed through the corners of the path.
    auto at = part.position();
    double travel = SPEED * p_dt.value();
    while (travel > 0.0 && !m_path.empty())
    {
        robotik::Vector3 const& goal = m_path.front();
        double const dx = goal.x - static_cast<double>(at.x);
        double const dy = goal.y - static_cast<double>(at.y);
        double const left = std::hypot(dx, dy);
        if (left <= travel)
        {
            at.x = static_cast<float>(goal.x);
            at.y = static_cast<float>(goal.y);
            travel -= left;
            m_path.erase(m_path.begin());
        }
        else
        {
            at.x += static_cast<float>(travel * dx / left);
            at.y += static_cast<float>(travel * dy / left);
            travel = 0.0;
        }
    }
    part.position(at);
    if (m_path.empty())
    {
        m_delivered = m_part;
        m_running = false;
    }
}

// ----------------------------------------------------------------------------
ConveySkill::ConveySkill(Conveyor& p_conveyor,
                         std::string p_part,
                         std::string p_feeder,
                         std::string p_zone)
    : m_conveyor(p_conveyor),
      m_part(std::move(p_part)),
      m_feeder(std::move(p_feeder)),
      m_zone(std::move(p_zone))
{
}

robotik::Status ConveySkill::tick(robotik::RobotContext& p_context,
                                  robotik::Seconds /*p_dt*/)
{
    if (!m_started)
    {
        // The belt runs in the world frame, as the shared beliefs.
        robotik::WorldModel const& beliefs = p_context.world_model;
        robotik::WorldObject const* part = beliefs.find(m_part);
        robotik::WorldObject const* feeder = beliefs.find(m_feeder);
        robotik::WorldObject const* zone = beliefs.find(m_zone);
        if (part == nullptr || feeder == nullptr || zone == nullptr)
        {
            m_why = (part == nullptr     ? m_part
                     : feeder == nullptr ? m_feeder
                                         : m_zone) +
                    " is not in the beliefs";
            return robotik::Status::FAILURE;
        }
        // Off the feeder, across to the lane of the zone, down the lane.
        double const transfer =
            feeder->position.x - 0.5 * feeder->size.x - TRANSFER;
        m_conveyor.carry(
            m_part,
            { robotik::Vector3(transfer, part->position.y, 0.0),
              robotik::Vector3(transfer, zone->position.y, 0.0),
              robotik::Vector3(zone->position.x, zone->position.y, 0.0) });
        m_started = true;
    }
    return m_conveyor.delivered() == m_part ? robotik::Status::SUCCESS
                                            : robotik::Status::RUNNING;
}

void ConveySkill::cancel(robotik::RobotContext& /*p_context*/)
{
    m_conveyor.stop();
}

} // namespace dual_arm
