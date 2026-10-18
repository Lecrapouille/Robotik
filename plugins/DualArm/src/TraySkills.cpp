// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "TraySkills.hpp"

#include "ArmSkills.hpp"

#include "Robotik/Perception/WorldModel.hpp"
#include "Robotik/Robot/Robot.hpp"
#include "Robotik/Runtime/RobotContext.hpp"

#include <algorithm>
#include <cmath>

namespace dual_arm
{

using robotik::Status;

/// Tool close enough to its visit point (m).
static constexpr double kVisitTolerance = 0.015;
/// The tool first comes this high above its visit point (m).
static constexpr double kApproachHeight = 0.20;
/// Close enough to the approach point to go down (m).
static constexpr double kApproachTolerance = 0.03;

/// @brief Puts the tool of the robot of @p_context at @p_target (base
/// frame): one inverse kinematics from the current joints, then each joint
/// servoed to its solution. False when there is no solution.
static bool servo(robotik::RobotContext& p_context, robotik::Pose const& p_target)
{
    robotik::Robot& robot = p_context.robot;
    auto const solution = robot.inverseKinematics(robot.tool(), p_target);
    if (!solution)
    {
        return false;
    }
    robotik::JointSet& joints = robot.joints();
    for (robotik::JointId id = 0; id < joints.size(); ++id)
    {
        joints.moveTo(id, (*solution)[id]);
    }
    return true;
}

/// @brief Minimum-jerk profile: 0 at 0, 1 at 1, no speed nor acceleration
/// at both ends.
static double minimumJerk(double p_s)
{
    double const s = std::clamp(p_s, 0.0, 1.0);
    return s * s * s * (10.0 - 15.0 * s + 6.0 * s * s);
}

// ----------------------------------------------------------------------------
PathSkill::PathSkill(robotik::Vector3 const& p_offset, Seconds p_duration)
    : m_offset(p_offset), m_duration(p_duration)
{
}

PathSkill::PathSkill(std::string p_object, Seconds p_duration)
    : m_object(std::move(p_object)), m_duration(p_duration)
{
}

Status PathSkill::tick(robotik::RobotContext& p_context, Seconds p_dt)
{
    robotik::Robot& robot = p_context.robot;
    if (!m_started)
    {
        m_from = robot.framePose(robot.tool());
        m_to = m_from.position + m_offset;
        if (!m_object.empty())
        {
            robotik::WorldObject const* object = p_context.world_model.find(m_object);
            if (object == nullptr)
            {
                m_why = m_object + " is not in the beliefs";
                return Status::FAILURE;
            }
            robotik::Vector3 const above = inBase(p_context, object->position);
            m_to = robotik::Vector3(above.x, above.y, m_from.position.z);
        }
        m_elapsed = Seconds{};
        m_started = true;
    }
    m_elapsed += p_dt;
    double const s = m_elapsed.value() / std::max(1e-3, m_duration.value());
    double const k = minimumJerk(s);
    robotik::Pose const target{ m_from.position + k * (m_to - m_from.position), m_from.rotation };
    if (!servo(p_context, target))
    {
        m_why = "no joint solution on the way";
        return Status::FAILURE;
    }
    double const left = (robot.framePose(robot.tool()).position - m_to).norm();
    return s >= 1.0 && left < kVisitTolerance ? Status::SUCCESS : Status::RUNNING;
}

void PathSkill::cancel(robotik::RobotContext& p_context)
{
    p_context.robot.joints().hold();
}

// ----------------------------------------------------------------------------
VisitSkill::VisitSkill(std::string p_glass,
                       std::string p_tray,
                       Length p_clearance,
                       Seconds p_dwell,
                       Seconds p_timeout)
    : m_glass(std::move(p_glass)),
      m_tray(std::move(p_tray)),
      m_clearance(p_clearance),
      m_dwell(p_dwell),
      m_timeout(p_timeout)
{
}

Status VisitSkill::tick(robotik::RobotContext& p_context, Seconds p_dt)
{
    // The arm that carries the tray, found by what its cup holds.
    robotik::Robot const* holder = nullptr;
    for (robotik::Robot const* robot : p_context.team)
    {
        holder = holder == nullptr && heldObject(*robot) == m_tray ? robot : holder;
    }
    if (holder == nullptr)
    {
        m_why = "no arm holds " + m_tray;
        return Status::FAILURE;
    }
    robotik::Pose const carrier = holder->worldPose(holder->tool());
    if (!m_started)
    {
        robotik::WorldObject const* glass = p_context.world_model.find(m_glass);
        if (glass == nullptr)
        {
            m_why = m_glass + " is not in the beliefs";
            return Status::FAILURE;
        }
        // Shared beliefs: the world frame, as the pose of the carrier.
        robotik::Vector3 const point(glass->position.x, glass->position.y,
                                     glass->top() + m_clearance.value());
        m_offset = carrier.inverse() * point;
        m_holder = holder->id();
        m_started = true;
    }
    robotik::Vector3 const visit = carrier * m_offset;
    robotik::Vector3 const above = visit + robotik::Vector3(0.0, 0.0, kApproachHeight);
    robotik::Robot const& self = p_context.robot;
    if (!m_descending && (self.worldPose(self.tool()).position - above).norm() < kApproachTolerance)
    {
        m_descending = true;
    }
    robotik::Vector3 const goal = m_descending ? visit : above;
    robotik::Vector3 const at = inBase(p_context, goal);
    if (!servo(p_context, { at, pointingDown(at) }))
    {
        m_why = "no joint solution above " + m_glass;
        return Status::FAILURE;
    }

    robotik::Robot const& robot = p_context.robot;
    double const error = (robot.worldPose(robot.tool()).position - goal).norm();
    m_still = m_descending && error < kVisitTolerance ? m_still + p_dt : Seconds{};
    m_elapsed += p_dt;
    if (m_still >= m_dwell)
    {
        return Status::SUCCESS;
    }
    if (m_elapsed > m_timeout)
    {
        m_why = "the tool did not keep above " + m_glass + " (error " +
                std::to_string(static_cast<int>(error * 1000.0)) + " mm)";
        return Status::FAILURE;
    }
    return Status::RUNNING;
}

void VisitSkill::cancel(robotik::RobotContext& p_context)
{
    p_context.robot.joints().hold();
}

} // namespace dual_arm
