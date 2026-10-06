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

#define DEFAULT_SLEW_RATE 1.0

namespace robotik
{

JointId JointSet::append(std::string p_name,
                         JointType p_type,
                         Bounds const& p_bounds,
                         compages::world::Entity p_link)
{
    if (find(p_name) != NO_JOINT)
    {
        throw std::invalid_argument("Duplicated joint '" + p_name + "'");
    }
    m_names.push_back(std::move(p_name));
    m_links.push_back(p_link);
    m_gains.emplace_back();
    m_homes.push_back(0.0);
    m_types.push_back(p_type);
    m_modes.push_back(JointMode::Disabled);
    m_bounds.push_back(p_bounds);
    m_positions.push_back(0.0);
    m_velocities.push_back(0.0);
    m_efforts.push_back(0.0);
    m_targets.push_back(0.0);
    m_references.push_back(0.0);
    return static_cast<JointId>(m_names.size() - 1u);
}

JointId JointSet::add(std::string p_name, compages::world::Entity p_link)
{
    if (auto const* joint = p_link.find<compages::world::RevoluteJoint>())
    {
        auto const& state = joint->state;
        bool const continuous =
            std::isinf(state.position.min.value()) &&
            std::isinf(state.position.max.value());
        return append(std::move(p_name),
                      continuous ? JointType::Continuous : JointType::Revolute,
                      { state.position.min.value(),
                        state.position.max.value(),
                        state.velocity.max.value(),
                        state.effort.max.value() },
                      p_link);
    }
    if (auto const* joint = p_link.find<compages::world::PrismaticJoint>())
    {
        auto const& state = joint->state;
        return append(std::move(p_name),
                      JointType::Prismatic,
                      { state.position.min.value(),
                        state.position.max.value(),
                        state.velocity.max.value(),
                        state.effort.max.value() },
                      p_link);
    }
    throw std::invalid_argument("Link of joint '" + p_name +
                                "' has no revolute nor prismatic joint");
}

Revolute JointSet::add(std::string p_name,
                       RevoluteLimits const& p_limits,
                       bool p_continuous)
{
    return { append(std::move(p_name),
                    p_continuous ? JointType::Continuous : JointType::Revolute,
                    { p_limits.lower.value(),
                      p_limits.upper.value(),
                      p_limits.velocity.value(),
                      p_limits.effort.value() },
                    {}) };
}

Prismatic JointSet::add(std::string p_name, PrismaticLimits const& p_limits)
{
    return { append(std::move(p_name),
                    JointType::Prismatic,
                    { p_limits.lower.value(),
                      p_limits.upper.value(),
                      p_limits.velocity.value(),
                      p_limits.effort.value() },
                    {}) };
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

Revolute JointSet::revolute(JointId p_id) const
{
    if (p_id >= size() || m_types[p_id] == JointType::Prismatic)
    {
        throw std::invalid_argument("Joint " + std::to_string(p_id) +
                                    " is not revolute");
    }
    return { p_id };
}

Prismatic JointSet::prismatic(JointId p_id) const
{
    if (p_id >= size() || m_types[p_id] != JointType::Prismatic)
    {
        throw std::invalid_argument("Joint " + std::to_string(p_id) +
                                    " is not prismatic");
    }
    return { p_id };
}

Revolute JointSet::revolute(std::string_view p_name) const
{
    return revolute(require(p_name));
}

Prismatic JointSet::prismatic(std::string_view p_name) const
{
    return prismatic(require(p_name));
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
        Bounds const& bounds = m_bounds[i];
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
                double const speed = std::isfinite(bounds.velocity)
                                         ? bounds.velocity * gains.speed_ratio
                                         : DEFAULT_SLEW_RATE;
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
        m_efforts[i] = std::clamp(effort, -bounds.effort, bounds.effort);
    }
}

void JointSet::publish() const
{
    for (std::size_t i = 0; i < m_names.size(); ++i)
    {
        compages::world::Entity link = m_links[i];
        if (!link)
        {
            continue;
        }
        if (auto* joint = link.find<compages::world::RevoluteJoint>())
        {
            joint->state.position.value = Radians(m_positions[i]);
            joint->state.velocity.value = AngularVelocity(m_velocities[i]);
            joint->state.effort.value = Torque(m_efforts[i]);
        }
        else if (auto* joint = link.find<compages::world::PrismaticJoint>())
        {
            joint->state.position.value = Length(m_positions[i]);
            joint->state.velocity.value = LinearVelocity(m_velocities[i]);
            joint->state.effort.value = Force(m_efforts[i]);
        }
    }
}

} // namespace robotik
