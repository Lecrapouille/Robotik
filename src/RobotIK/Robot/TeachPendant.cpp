// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "RobotIK/Robot/TeachPendant.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <optional>
#include <utility>

namespace robotik
{

// ----------------------------------------------------------------------------
/// Keeps a joint command inside the limits of the joint, when it has some
/// (a wheel turns without limits).
/// @tparam Position @c Radians for a joint that turns, @c Length for one
///   that slides: the handle and the value have the same unit.
// ----------------------------------------------------------------------------
template <class Position>
static Position clampJoint(JointSet const& p_joints,
                           JointHandle<Position> p_joint,
                           Position p_value)
{
    JointLimits<Position> const limits = p_joints.limits(p_joint);
    return bounded(limits) ? std::clamp(p_value, limits.lower, limits.upper)
                           : p_value;
}

// ----------------------------------------------------------------------------
/// Where a jog of @p_delta takes a joint: from its target (or from where it
/// is, when it is not driven in position), inside its limits.
// ----------------------------------------------------------------------------
template <class Position>
static Position jogged(JointSet const& p_joints,
                       JointHandle<Position> p_joint,
                       Position p_delta)
{
    Position const from(p_joints.mode(p_joint.id) == JointMode::Position
                            ? p_joints.target(p_joint.id)
                            : p_joints.position(p_joint.id));
    return clampJoint(p_joints, p_joint, from + p_delta);
}

// ----------------------------------------------------------------------------
double TeachPendant::Polynomial::position(double p_t) const
{
    double const t2 = p_t * p_t;
    double const t3 = t2 * p_t;
    double const t4 = t3 * p_t;
    double const t5 = t4 * p_t;
    return a0 + a3 * t3 + a4 * t4 + a5 * t5;
}

// ----------------------------------------------------------------------------
std::vector<double> TeachPendant::command(Robot const& p_robot) const
{
    JointSet const& joints = p_robot.getJoints();
    std::vector<double> values(joints.size());
    for (JointId id = 0; id < joints.size(); ++id)
    {
        values[id] = joints.mode(id) == JointMode::Position
                         ? joints.target(id)
                         : joints.position(id);
    }
    return values;
}

// ----------------------------------------------------------------------------
bool TeachPendant::compatible(Robot const& p_robot,
                              Waypoint const& p_waypoint) const
{
    return p_waypoint.joints.size() == p_robot.getJoints().size() &&
           p_waypoint.joint_names == p_robot.getModel().joints;
}

// ----------------------------------------------------------------------------
void TeachPendant::apply(Robot& p_robot, std::vector<double> const& p_positions)
{
    JointSet& joints = p_robot.getJoints();
    std::size_t const count = std::min(joints.size(), p_positions.size());
    for (JointId id = 0; id < count; ++id)
    {
        if (joints.isPrismatic(id))
        {
            Prismatic const joint = joints.requirePrismatic(id);
            joints.moveTo(joint,
                          clampJoint(joints, joint, Length(p_positions[id])));
        }
        else
        {
            Revolute const joint = joints.requireRevolute(id);
            joints.moveTo(joint,
                          clampJoint(joints, joint, Radians(p_positions[id])));
        }
    }
}

// ----------------------------------------------------------------------------
void TeachPendant::begin(Robot const& p_robot, Waypoint const& p_goal)
{
    std::vector<double> const from = command(p_robot);
    m_duration = std::max(p_goal.duration.value(), 0.05);
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

// ----------------------------------------------------------------------------
bool TeachPendant::jogJoint(Robot& p_robot, Revolute p_joint, Radians p_delta)
{
    m_playing = false;
    JointSet& joints = p_robot.getJoints();
    if (!p_joint || p_joint.id >= joints.size())
    {
        m_error = "Unknown joint";
        return false;
    }
    joints.moveTo(p_joint, jogged(joints, p_joint, p_delta));
    m_error.clear();
    return true;
}

// ----------------------------------------------------------------------------
bool TeachPendant::jogJoint(Robot& p_robot, Prismatic p_joint, Length p_delta)
{
    m_playing = false;
    JointSet& joints = p_robot.getJoints();
    if (!p_joint || p_joint.id >= joints.size())
    {
        m_error = "Unknown joint";
        return false;
    }
    joints.moveTo(p_joint, jogged(joints, p_joint, p_delta));
    m_error.clear();
    return true;
}

// ----------------------------------------------------------------------------
std::string const& TeachPendant::getFrame(Robot const& p_robot) const
{
    return m_frame.empty() ? p_robot.getTool() : m_frame;
}

// ----------------------------------------------------------------------------
IkGoal TeachPendant::getJogGoal(Robot const& p_robot, bool p_rotation) const
{
    std::string const& moved = getFrame(p_robot);
    if (moved.empty() || p_robot.getKinematics().requireChainDofs(moved) >= 6U)
    {
        return IkGoal::Whole;
    }
    return p_rotation ? IkGoal::Orientation : IkGoal::Position;
}

// ----------------------------------------------------------------------------
std::array<bool, 6> TeachPendant::getJogAxes(Robot const& p_robot) const
{
    std::string const& moved = getFrame(p_robot);
    if (moved.empty())
    {
        return {};
    }
    // Position and Orientation give the same flags: each block alone.
    return p_robot.getKinematics().requireMovableAxes(
        moved,
        getJogGoal(p_robot, false) == IkGoal::Whole ? IkGoal::Whole
                                                    : IkGoal::Position);
}

// ----------------------------------------------------------------------------
bool TeachPendant::jogTool(Robot& p_robot,
                           Vector3m const& p_translation,
                           Vector3rad const& p_rotation)
{
    m_playing = false;
    std::string const& moved = getFrame(p_robot);
    if (moved.empty())
    {
        m_error = "Robot has no tool frame";
        return false;
    }

    // The IK and the checks below work in plain numbers (m, rad).
    Vector3 const shift(p_translation.x.value(),
                        p_translation.y.value(),
                        p_translation.z.value());
    Vector3 const angles(
        p_rotation.x.value(), p_rotation.y.value(), p_rotation.z.value());
    Pose target = p_robot.requireFramePose(moved);
    target.position.x += shift.x;
    target.position.y += shift.y;
    target.position.z += shift.z;
    bool const turn =
        std::abs(angles.x) + std::abs(angles.y) + std::abs(angles.z) > 0.0;
    if (turn)
    {
        target.rotation =
            Quaternion::fromAngleAxis(p_rotation.x, Vector3(1.0, 0.0, 0.0)) *
            Quaternion::fromAngleAxis(p_rotation.y, Vector3(0.0, 1.0, 0.0)) *
            Quaternion::fromAngleAxis(p_rotation.z, Vector3(0.0, 0.0, 1.0)) *
            target.rotation;
    }

    IkGoal const goal = getJogGoal(p_robot, turn);
    if (goal != IkGoal::Whole)
    {
        // Few joints: the move goes to the closest point, so it must be
        // mostly along the asked axes.
        std::array<bool, 6> const axes = getJogAxes(p_robot);
        Vector3 const& asked = turn ? angles : shift;
        std::size_t const first = turn ? 3U : 0U;
        if ((asked.x != 0.0 && !axes[first]) ||
            (asked.y != 0.0 && !axes[first + 1U]) ||
            (asked.z != 0.0 && !axes[first + 2U]))
        {
            m_error = moved + " cannot move along this axis: its joints do "
                              "not allow it";
            return false;
        }
    }
    std::optional<std::vector<double>> const solution =
        p_robot.inverseKinematics(moved, target, goal);
    if (!solution)
    {
        m_error = moved +
                  " cannot reach this pose: its joints do not allow this move "
                  "(IK did not converge)";
        return false;
    }
    apply(p_robot, *solution);
    m_error.clear();
    return true;
}

// ----------------------------------------------------------------------------
std::size_t TeachPendant::record(Robot const& p_robot,
                                 std::string p_label,
                                 Seconds p_duration)
{
    Waypoint waypoint;
    waypoint.label = std::move(p_label);
    if (waypoint.label.empty())
    {
        waypoint.label = "wp " + std::to_string(m_waypoints.size());
    }
    waypoint.joints.assign(p_robot.getJoints().positions().begin(),
                           p_robot.getJoints().positions().end());
    waypoint.joint_names = p_robot.getModel().joints;
    waypoint.pose = getFrame(p_robot).empty()
                        ? Pose{}
                        : p_robot.requireFramePose(getFrame(p_robot));
    waypoint.duration = Seconds(std::max(p_duration.value(), 0.05));
    m_waypoints.push_back(std::move(waypoint));
    m_error.clear();
    return m_waypoints.size() - 1U;
}

// ----------------------------------------------------------------------------
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

// ----------------------------------------------------------------------------
void TeachPendant::clear()
{
    m_playing = false;
    m_cursor = -1;
    m_waypoints.clear();
    m_polynomials.clear();
}

// ----------------------------------------------------------------------------
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

// ----------------------------------------------------------------------------
bool TeachPendant::play(Robot& p_robot, bool p_loop)
{
    if (m_waypoints.empty())
    {
        m_error = "No waypoint";
        return false;
    }
    for (Waypoint const& waypoint : m_waypoints)
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

// ----------------------------------------------------------------------------
void TeachPendant::stop(Robot& p_robot)
{
    m_playing = false;
    m_cursor = -1;
    p_robot.getJoints().hold();
}

// ----------------------------------------------------------------------------
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
