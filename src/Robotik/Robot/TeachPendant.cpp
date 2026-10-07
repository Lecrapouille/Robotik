// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "Robotik/Robot/TeachPendant.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <optional>
#include <utility>

namespace robotik
{

namespace
{

double clampJoint(JointSet const& p_joints, JointId p_id, double p_value)
{
    if (p_joints.isPrismatic(p_id))
    {
        PrismaticLimits const limits = p_joints.limits(p_joints.prismatic(p_id));
        if (limits.bounded())
        {
            return std::clamp(p_value, limits.lower.value(), limits.upper.value());
        }
    }
    else
    {
        RevoluteLimits const limits = p_joints.limits(p_joints.revolute(p_id));
        if (limits.bounded())
        {
            return std::clamp(p_value, limits.lower.value(), limits.upper.value());
        }
    }
    return p_value;
}

} // namespace

double TeachPendant::Polynomial::position(double p_t) const
{
    double const t2 = p_t * p_t;
    double const t3 = t2 * p_t;
    double const t4 = t3 * p_t;
    double const t5 = t4 * p_t;
    return a0 + a3 * t3 + a4 * t4 + a5 * t5;
}

std::vector<double> TeachPendant::command(Robot const& p_robot) const
{
    JointSet const& joints = p_robot.joints();
    std::vector<double> values(joints.size());
    for (JointId id = 0; id < joints.size(); ++id)
    {
        values[id] = joints.mode(id) == JointMode::Position ? joints.target(id)
                                                            : joints.position(id);
    }
    return values;
}

bool TeachPendant::compatible(Robot const& p_robot,
                              TeachWaypoint const& p_waypoint) const
{
    return p_waypoint.joints.size() == p_robot.joints().size();
}

void TeachPendant::apply(Robot& p_robot, std::vector<double> const& p_positions)
{
    JointSet& joints = p_robot.joints();
    std::size_t const count = std::min(joints.size(), p_positions.size());
    for (JointId id = 0; id < count; ++id)
    {
        joints.moveTo(id, clampJoint(joints, id, p_positions[id]));
    }
}

void TeachPendant::begin(Robot const& p_robot, TeachWaypoint const& p_goal)
{
    std::vector<double> const from = command(p_robot);
    m_duration = std::max(p_goal.duration, 0.05);
    m_time = 0.0;
    m_polynomials.clear();
    m_polynomials.reserve(from.size());
    double const t3 = m_duration * m_duration * m_duration;
    double const t4 = t3 * m_duration;
    double const t5 = t4 * m_duration;
    for (std::size_t i = 0; i < from.size(); ++i)
    {
        double const delta = p_goal.joints[i] - from[i];
        Polynomial polynomial;
        polynomial.a0 = from[i];
        polynomial.a3 = 10.0 * delta / t3;
        polynomial.a4 = -15.0 * delta / t4;
        polynomial.a5 = 6.0 * delta / t5;
        m_polynomials.push_back(polynomial);
    }
}

bool TeachPendant::jogJoint(Robot& p_robot, JointId p_joint, double p_delta)
{
    m_playing = false;
    JointSet& joints = p_robot.joints();
    if (p_joint >= joints.size())
    {
        m_error = "Unknown joint";
        return false;
    }
    double const from = joints.mode(p_joint) == JointMode::Position
                            ? joints.target(p_joint)
                            : joints.position(p_joint);
    joints.moveTo(p_joint, clampJoint(joints, p_joint, from + p_delta));
    m_error.clear();
    return true;
}

bool TeachPendant::jogTool(Robot& p_robot,
                           Vector3 const& p_translation,
                           Vector3 const& p_rotation)
{
    m_playing = false;
    if (p_robot.tool().empty())
    {
        m_error = "Robot has no tool frame";
        return false;
    }

    Pose target = p_robot.framePose(p_robot.tool());
    target.position.x += p_translation.x;
    target.position.y += p_translation.y;
    target.position.z += p_translation.z;
    if (std::abs(p_rotation.x) + std::abs(p_rotation.y) + std::abs(p_rotation.z) >
        0.0)
    {
        target.rotation = axisAngle(Vector3(1.0, 0.0, 0.0), p_rotation.x) *
                          axisAngle(Vector3(0.0, 1.0, 0.0), p_rotation.y) *
                          axisAngle(Vector3(0.0, 0.0, 1.0), p_rotation.z) *
                          target.rotation;
    }

    std::optional<std::vector<double>> const solution =
        p_robot.inverseKinematics(p_robot.tool(), target);
    if (!solution)
    {
        m_error = "IK did not converge";
        return false;
    }
    apply(p_robot, *solution);
    m_error.clear();
    return true;
}

std::size_t TeachPendant::record(Robot const& p_robot,
                                 std::string p_label,
                                 double p_duration)
{
    TeachWaypoint waypoint;
    waypoint.label = std::move(p_label);
    if (waypoint.label.empty())
    {
        waypoint.label = "wp " + std::to_string(m_waypoints.size());
    }
    waypoint.joints.assign(p_robot.joints().positions().begin(),
                           p_robot.joints().positions().end());
    waypoint.pose =
        p_robot.tool().empty() ? Pose{} : p_robot.framePose(p_robot.tool());
    waypoint.duration = std::max(p_duration, 0.05);
    m_waypoints.push_back(std::move(waypoint));
    m_error.clear();
    return m_waypoints.size() - 1u;
}

void TeachPendant::erase(std::size_t p_index)
{
    if (p_index >= m_waypoints.size())
    {
        return;
    }
    m_playing = false;
    m_cursor = -1;
    m_waypoints.erase(m_waypoints.begin() +
                      static_cast<std::ptrdiff_t>(p_index));
}

void TeachPendant::clear()
{
    m_playing = false;
    m_cursor = -1;
    m_waypoints.clear();
    m_polynomials.clear();
}

bool TeachPendant::goTo(Robot& p_robot, std::size_t p_index)
{
    if (p_index >= m_waypoints.size())
    {
        m_error = "No such waypoint";
        return false;
    }
    if (!compatible(p_robot, m_waypoints[p_index]))
    {
        m_error = "Waypoint does not match this robot";
        return false;
    }
    m_sequence = false;
    m_cursor = static_cast<int>(p_index);
    begin(p_robot, m_waypoints[p_index]);
    m_playing = true;
    m_error.clear();
    return true;
}

bool TeachPendant::play(Robot& p_robot, bool p_loop)
{
    if (m_waypoints.empty())
    {
        m_error = "No waypoint";
        return false;
    }
    for (TeachWaypoint const& waypoint : m_waypoints)
    {
        if (!compatible(p_robot, waypoint))
        {
            m_error = "Waypoint does not match this robot";
            return false;
        }
    }
    m_loop = p_loop;
    m_sequence = true;
    m_cursor = 0;
    begin(p_robot, m_waypoints[0]);
    m_playing = true;
    m_error.clear();
    return true;
}

void TeachPendant::stop(Robot& p_robot)
{
    m_playing = false;
    m_cursor = -1;
    p_robot.joints().hold();
}

void TeachPendant::update(Robot& p_robot, Seconds p_dt)
{
    if (!m_playing)
    {
        return;
    }
    m_time += p_dt.value();
    double const t = std::min(m_time, m_duration);
    std::vector<double> positions(m_polynomials.size());
    for (std::size_t i = 0; i < m_polynomials.size(); ++i)
    {
        positions[i] = m_polynomials[i].position(t);
    }
    apply(p_robot, positions);
    if (m_time + 1e-12 < m_duration)
    {
        return;
    }
    if (!m_sequence)
    {
        m_playing = false;
        return;
    }
    ++m_cursor;
    if (m_cursor >= static_cast<int>(m_waypoints.size()))
    {
        if (!m_loop)
        {
            m_playing = false;
            m_cursor = -1;
            return;
        }
        m_cursor = 0;
    }
    begin(p_robot, m_waypoints[static_cast<std::size_t>(m_cursor)]);
}

} // namespace robotik
