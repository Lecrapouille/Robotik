// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "WalkSkills.hpp"

#include "Robotik/Robot/Robot.hpp"
#include "Robotik/Runtime/RobotContext.hpp"
#include "Robotik/Sensors/Imu.hpp"

#include <algorithm>

/// Gravity along the IMU z axis above which the robot lies down (upright: -1).
#define FALLEN_GRAVITY_Z -0.5

/// Damping mode of the LimX controllers: no stiffness, a viscous brake
/// (JointGains::velocity_kp = kd). Without its IMU the robot cannot balance:
/// it sags to the ground instead of freezing in a stiff pose.
static void damp(robotik::JointSet& p_joints,
                 std::vector<robotik::JointId> const& p_ids)
{
    for (robotik::JointId const id : p_ids)
    {
        p_joints.spin(id, 0.0);
    }
}

// ****************************************************************************
StandSkill::StandSkill(WalkBody p_body,
                       std::vector<double> p_posture,
                       Seconds p_duration)
    : m_body(std::move(p_body)),
      m_posture(std::move(p_posture)),
      m_duration(p_duration)
{
}

void StandSkill::reset()
{
    m_elapsed = Seconds{};
}

robotik::Status StandSkill::tick(robotik::RobotContext& p_context, Seconds p_dt)
{
    robotik::JointSet& joints = p_context.robot.joints();
    for (std::size_t i = 0; i < m_body.joints.size(); ++i)
    {
        joints.moveTo(m_body.joints[i], m_posture[i]);
    }
    m_elapsed = m_elapsed + p_dt;
    return m_elapsed >= m_duration ? robotik::Status::SUCCESS
                                   : robotik::Status::RUNNING;
}

void StandSkill::cancel(robotik::RobotContext& p_context)
{
    p_context.robot.joints().hold();
}

// ****************************************************************************
WalkSkill::WalkSkill(WalkBody p_body,
                     WalkPolicy& p_policy,
                     std::array<double, 3> const& p_command,
                     Seconds p_duration,
                     Seconds p_period)
    : m_body(std::move(p_body)),
      m_policy(p_policy),
      m_command(p_command),
      m_duration(p_duration),
      m_period(p_period),
      m_positions(m_body.joints.size()),
      m_velocities(m_body.joints.size())
{
}

void WalkSkill::reset()
{
    m_policy.reset();
    m_elapsed = Seconds{};
    m_wait = Seconds{};
}

robotik::Status WalkSkill::tick(robotik::RobotContext& p_context, Seconds p_dt)
{
    robotik::Robot& robot = p_context.robot;
    auto const* imu = robot.sensors().find<robotik::Imu>(m_body.imu);
    if (imu == nullptr || !imu->reading().valid)
    {
        damp(robot.joints(), m_body.joints);
        return robotik::Status::FAILURE;
    }
    robotik::ImuReading const& reading = imu->reading();
    robotik::Vector3 const gravity =
        reading.orientation.conjugate() * robotik::Vector3(0.0, 0.0, -1.0);
    if (gravity.z > FALLEN_GRAVITY_Z)
    {
        damp(robot.joints(), m_body.joints);
        return robotik::Status::FAILURE;
    }

    m_wait = m_wait - p_dt;
    if (m_wait.value() <= 1e-9)
    {
        m_wait = m_wait + m_period;
        robotik::JointSet& joints = robot.joints();
        for (std::size_t i = 0; i < m_body.joints.size(); ++i)
        {
            m_positions[i] = joints.position(m_body.joints[i]);
            m_velocities[i] = joints.velocity(m_body.joints[i]);
        }
        std::array<double, 3> command{};
        std::array<double, 3> const& limit = m_policy.contract().max_command;
        for (std::size_t i = 0; i < command.size(); ++i)
        {
            command[i] = std::clamp(m_command[i], -limit[i], limit[i]);
        }
        WalkInput const input{ reading.orientation,
                               reading.angular_velocity,
                               command,
                               m_positions,
                               m_velocities };
        std::span<double const> const targets = m_policy.act(input);
        for (std::size_t i = 0; i < m_body.joints.size(); ++i)
        {
            joints.moveTo(m_body.joints[i], targets[i]);
        }
    }

    m_elapsed = m_elapsed + p_dt;
    bool const done = m_duration.value() > 0.0 && m_elapsed >= m_duration;
    return done ? robotik::Status::SUCCESS : robotik::Status::RUNNING;
}

void WalkSkill::cancel(robotik::RobotContext& p_context)
{
    p_context.robot.joints().hold();
}
