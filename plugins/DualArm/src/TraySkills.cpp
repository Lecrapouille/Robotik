// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "TraySkills.hpp"

#include "ArmSkills.hpp"

#include "Robotik/Perception/WorldModel.hpp"
#include "Robotik/Robot/Actuators.hpp"
#include "Robotik/Robot/Robot.hpp"
#include "Robotik/Runtime/RobotContext.hpp"
#include "Robotik/Sensors/ForceTorqueSensor.hpp"

#include <algorithm>
#include <cmath>

namespace dual_arm
{

using robotik::Status;

/// Tool close enough to the end of a straight move (m).
static constexpr double kPathTolerance = 0.015;
/// The tool first comes this high above the start of the path (m): clear
/// of the glasses.
static constexpr double kApproachHeight = 0.12;
/// Close enough to the approach point to go down (m).
static constexpr double kApproachTolerance = 0.02;
/// On its way to the tray, the tool chases a point at most this far ahead
/// of it: small steps for the inverse kinematics (m).
static constexpr double kApproachStep = 0.05;
/// Vertical speed of the tool on its way down to the tray and back up (m/s).
static constexpr double kDescentSpeed = 0.06;
/// Slower when it looks for the contact, the last centimetres (m/s).
static constexpr double kSearchSpeed = 0.01;
/// Height above the believed top where the search starts (m).
static constexpr double kSearchHeight = 0.01;
/// The tool touches when it feels this force (N).
static constexpr double kTouchForce = 1.0;
/// Speed of the tool per newton of force error, keeping the force (m/s/N).
static constexpr double kAdmittance = 0.002;
/// The tool does not go deeper than this under the believed top: no tray
/// there (m).
static constexpr double kDeepest = 0.02;

/// @brief Puts the tool of the robot of @p_context at @p_target (base
/// frame): one inverse kinematics from the current joints, then each joint
/// of the arm servoed to its solution. A tool joint (the spindle of a
/// drill) is left to its own motor. False when there is no solution.
static bool servo(robotik::RobotContext& p_context,
                  robotik::Pose const& p_target)
{
    robotik::Robot& robot = p_context.robot;
    auto const solution = robot.inverseKinematics(robot.tool(), p_target);
    if (!solution)
    {
        return false;
    }
    robotik::JointGroup const* arm =
        robot.actuators().first<robotik::JointGroup>();
    robotik::JointSet& joints = robot.joints();
    for (robotik::JointId id = 0; id < joints.size(); ++id)
    {
        if (arm == nullptr || arm->contains(id))
        {
            joints.moveTo(id, (*solution)[id]);
        }
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

PathSkill::PathSkill(std::string p_object, Length p_below, Seconds p_duration)
    : m_object(std::move(p_object)),
      m_below(p_below.value()),
      m_duration(p_duration)
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
            robotik::WorldObject const* object =
                p_context.world_model.find(m_object);
            if (object == nullptr)
            {
                m_why = m_object + " is not in the beliefs";
                return Status::FAILURE;
            }
            robotik::Vector3 const above = inBase(p_context, object->position);
            m_to = robotik::Vector3(above.x, above.y, m_from.position.z);
            if (m_below)
            {
                double const bottom = object->position.z - 0.5 * object->size.z;
                m_to.z = inBase(p_context,
                                robotik::Vector3(0.0, 0.0, bottom - *m_below))
                             .z;
            }
        }
        m_elapsed = Seconds{};
        m_started = true;
    }
    m_elapsed += p_dt;
    double const s = m_elapsed.value() / std::max(1e-3, m_duration.value());
    double const k = minimumJerk(s);
    robotik::Pose const target{ m_from.position + k * (m_to - m_from.position),
                                m_from.rotation };
    if (!servo(p_context, target))
    {
        m_why = "no joint solution on the way";
        return Status::FAILURE;
    }
    double const left = (robot.framePose(robot.tool()).position - m_to).norm();
    return s >= 1.0 && left < kPathTolerance ? Status::SUCCESS
                                             : Status::RUNNING;
}

void PathSkill::cancel(robotik::RobotContext& p_context)
{
    p_context.robot.joints().hold();
}

// ----------------------------------------------------------------------------
TraceSkill::TraceSkill(std::string p_tray,
                       std::vector<robotik::Vector3> p_path,
                       LinearVelocity p_speed,
                       Force p_force,
                       Seconds p_timeout)
    : m_tray(std::move(p_tray)),
      m_path(std::move(p_path)),
      m_speed(p_speed),
      m_force(p_force),
      m_timeout(p_timeout)
{
}

robotik::Vector3 TraceSkill::along(double p_travel) const
{
    double left = p_travel;
    for (std::size_t i = 1; i < m_offsets.size(); ++i)
    {
        robotik::Vector3 const step = m_offsets[i] - m_offsets[i - 1];
        double const length = step.norm();
        if (left <= length && length > 0.0)
        {
            return m_offsets[i - 1] + (left / length) * step;
        }
        left -= length;
    }
    return m_offsets.back();
}

Status TraceSkill::tick(robotik::RobotContext& p_context, Seconds p_dt)
{
    // The arm that carries the tray, found by what its cup holds.
    robotik::Robot const* holder = nullptr;
    for (robotik::Robot const* robot : p_context.team)
    {
        holder =
            holder == nullptr && heldObject(*robot) == m_tray ? robot : holder;
    }
    if (holder == nullptr)
    {
        m_why = "no arm holds " + m_tray;
        return Status::FAILURE;
    }
    robotik::Pose const carrier = holder->worldPose(holder->tool());
    auto const* sensor =
        p_context.robot.sensors().first<robotik::ForceTorqueSensor>();
    if (sensor == nullptr || !sensor->reading().valid)
    {
        m_why = sensor == nullptr ? "no force sensor on the robot"
                                  : "the force sensor gives nothing";
        return Status::FAILURE;
    }
    if (m_offsets.empty())
    {
        robotik::WorldObject const* tray = p_context.world_model.find(m_tray);
        if (tray == nullptr || m_path.empty())
        {
            m_why = tray == nullptr ? m_tray + " is not in the beliefs"
                                    : "the path is empty";
            return Status::FAILURE;
        }
        // Shared beliefs: the world frame, as the pose of the carrier.
        for (robotik::Vector3 const& point : m_path)
        {
            m_offsets.push_back(carrier.inverse() * (tray->position + point));
        }
        m_height = kApproachHeight;
    }

    // Force of the contact along the tool, pushing it back (N): the tool
    // points down, the tray pushes it up.
    double const felt = (sensor->reading().wrench.force - m_tare).z;

    // Where the tool goes now: over the path, or on it and pressing.
    double const dt = p_dt.value();
    robotik::Robot const& self = p_context.robot;
    robotik::Vector3 const tool = self.worldPose(self.tool()).position;
    switch (m_phase)
    {
        case Phase::Approach:
        {
            robotik::Vector3 const above = carrier * m_offsets.front() +
                                           robotik::Vector3(0.0, 0.0, m_height);
            if ((tool - above).norm() < kApproachTolerance)
            {
                // Nothing touches yet: what the sensor reads is the tool.
                m_tare = sensor->reading().wrench.force;
                m_phase = Phase::Descend;
            }
            break;
        }
        case Phase::Descend:
            m_height -=
                (m_height > kSearchHeight ? kDescentSpeed : kSearchSpeed) * dt;
            if (felt > kTouchForce)
            {
                m_phase = Phase::Follow;
            }
            else if (m_height < -kDeepest)
            {
                m_why = "the tool felt no " + m_tray + " under it";
                return Status::FAILURE;
            }
            break;
        case Phase::Follow:
        {
            double const length = [this]
            {
                double total = 0.0;
                for (std::size_t i = 1; i < m_offsets.size(); ++i)
                {
                    total += (m_offsets[i] - m_offsets[i - 1]).norm();
                }
                return total;
            }();
            // Too much force: up; too little: down.
            m_height += std::clamp(kAdmittance * (felt - m_force.value()),
                                   -kSearchSpeed,
                                   kSearchSpeed) *
                        dt;
            m_height = std::max(m_height, -kDeepest);
            m_travel = std::min(length, m_travel + m_speed.value() * dt);
            m_phase = m_travel >= length ? Phase::Lift : m_phase;
            break;
        }
        case Phase::Lift:
            m_height = std::min(kApproachHeight, m_height + kDescentSpeed * dt);
            break;
    }
    robotik::Vector3 goal =
        carrier * along(m_travel) + robotik::Vector3(0.0, 0.0, m_height);
    if (m_phase == Phase::Approach && (goal - tool).norm() > kApproachStep)
    {
        goal = tool + (kApproachStep / (goal - tool).norm()) * (goal - tool);
    }
    robotik::Vector3 const at = inBase(p_context, goal);
    if (!servo(p_context, { at, pointingDown(at) }))
    {
        m_why = "no joint solution on the path";
        return Status::FAILURE;
    }

    if (m_phase == Phase::Lift && m_height >= kApproachHeight)
    {
        return Status::SUCCESS;
    }
    m_elapsed += p_dt;
    if (m_elapsed > m_timeout)
    {
        m_why = "the path on " + m_tray + " is not done after " +
                std::to_string(static_cast<int>(m_timeout.value())) + " s";
        return Status::FAILURE;
    }
    return Status::RUNNING;
}

void TraceSkill::cancel(robotik::RobotContext& p_context)
{
    p_context.robot.joints().hold();
}

} // namespace dual_arm
