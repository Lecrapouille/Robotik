// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "Robotik/Robot/Joints.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace robotik
{

JointId JointSet::add(std::string p_name,
                      JointType p_type,
                      JointLimits const& p_limits,
                      compages::world::Entity p_link)
{
    if (find(p_name) != NO_JOINT)
    {
        throw std::invalid_argument("Duplicated joint '" + p_name + "'");
    }
    m_names.push_back(std::move(p_name));
    m_links.push_back(p_link);
    m_gains.emplace_back();
    m_limits.push_back(p_limits);
    m_homes.push_back(0.0);
    m_types.push_back(p_type);
    m_modes.push_back(JointMode::Disabled);
    m_positions.push_back(0.0);
    m_velocities.push_back(0.0);
    m_efforts.push_back(0.0);
    m_targets.push_back(0.0);
    m_references.push_back(0.0);
    return static_cast<JointId>(m_names.size() - 1u);
}

JointId JointSet::find(std::string_view p_name) const
{
    auto const found = std::find(m_names.begin(), m_names.end(), p_name);
    return found == m_names.end()
               ? NO_JOINT
               : static_cast<JointId>(found - m_names.begin());
}

JointId JointSet::require(std::string_view p_name) const
{
    JointId const id = find(p_name);
    if (id == NO_JOINT)
    {
        throw std::invalid_argument("Unknown joint '" + std::string(p_name) +
                                    "'");
    }
    return id;
}

void JointSet::place(JointId p_id, double p_position)
{
    m_positions[p_id] = p_position;
    m_velocities[p_id] = 0.0;
    m_efforts[p_id] = 0.0;
    m_references[p_id] = p_position;
    m_targets[p_id] = p_position;
    m_modes[p_id] = JointMode::Position;
}

void JointSet::moveTo(JointId p_id, double p_position)
{
    if (m_modes[p_id] != JointMode::Position)
    {
        m_references[p_id] = m_positions[p_id];
    }
    m_modes[p_id] = JointMode::Position;
    m_targets[p_id] = p_position;
}

void JointSet::spin(JointId p_id, double p_velocity)
{
    m_modes[p_id] = JointMode::Velocity;
    m_targets[p_id] = p_velocity;
}

void JointSet::push(JointId p_id, double p_effort)
{
    m_modes[p_id] = JointMode::Effort;
    m_targets[p_id] = p_effort;
}

void JointSet::hold(JointId p_id)
{
    moveTo(p_id, m_positions[p_id]);
}

void JointSet::hold()
{
    for (JointId id = 0; id < m_names.size(); ++id)
    {
        hold(id);
    }
}

void JointSet::release(JointId p_id)
{
    m_modes[p_id] = JointMode::Disabled;
    m_targets[p_id] = 0.0;
}

bool JointSet::reached(JointId p_id, double p_tolerance) const
{
    return m_modes[p_id] == JointMode::Position &&
           std::abs(m_positions[p_id] - m_targets[p_id]) <= p_tolerance;
}

void JointSet::control(Seconds p_dt)
{
    double const dt = p_dt.value();
    std::size_t const count = m_names.size();
    for (std::size_t i = 0; i < count; ++i)
    {
        JointLimits const& limits = m_limits[i];
        JointGains const& gains = m_gains[i];
        double effort = 0.0;
        switch (m_modes[i])
        {
            case JointMode::Disabled:
                m_references[i] = m_positions[i];
                break;
            case JointMode::Position:
            {
                // Slewing the reference avoids a torque step on a new target.
                double const speed = limits.velocity > 0.0
                                         ? limits.velocity * gains.speed_ratio
                                         : 1.0;
                double const step = speed * dt;
                m_references[i] += std::clamp(
                    m_targets[i] - m_references[i], -step, step);
                effort = gains.kp * (m_references[i] - m_positions[i]) -
                         gains.kd * m_velocities[i];
                break;
            }
            case JointMode::Velocity:
                m_references[i] = m_positions[i];
                effort = gains.velocity_kp * (m_targets[i] - m_velocities[i]);
                break;
            case JointMode::Effort:
                m_references[i] = m_positions[i];
                effort = m_targets[i];
                break;
        }
        if (limits.effort > 0.0)
        {
            effort = std::clamp(effort, -limits.effort, limits.effort);
        }
        m_efforts[i] = effort;
    }
}

} // namespace robotik
