// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "DiffDrive.hpp"

#include <cmath>
#include <numbers>

#define WHEEL_TIME_CONSTANT_S 0.05

void DriveGeometry::twist(double p_left, double p_right, double& p_speed, double& p_yaw_rate) const
{
    double const left = sign * radius * p_left;
    double const right = sign * radius * p_right;
    p_speed = 0.5 * (left + right);
    p_yaw_rate = (right - left) / track;
}

void DriveGeometry::wheels(double p_speed, double p_yaw_rate, double& p_left, double& p_right) const
{
    p_left = (p_speed - 0.5 * track * p_yaw_rate) / (sign * radius);
    p_right = (p_speed + 0.5 * track * p_yaw_rate) / (sign * radius);
}

Pose2 DriveGeometry::integrate(Pose2 const& p_pose,
                               double p_speed,
                               double p_yaw_rate,
                               double p_dt) const
{
    double x = p_pose.x + axle * std::cos(p_pose.yaw);
    double y = p_pose.y + axle * std::sin(p_pose.yaw);
    double const yaw = p_pose.yaw + p_yaw_rate * p_dt;
    if (std::abs(p_yaw_rate) > 1e-9)
    {
        double const r = p_speed / p_yaw_rate;
        x += r * (std::sin(yaw) - std::sin(p_pose.yaw));
        y -= r * (std::cos(yaw) - std::cos(p_pose.yaw));
    }
    else
    {
        x += p_speed * p_dt * std::cos(p_pose.yaw);
        y += p_speed * p_dt * std::sin(p_pose.yaw);
    }
    return { x - axle * std::cos(yaw), y - axle * std::sin(yaw), std::remainder(yaw, 2.0 * std::numbers::pi) };
}

DiffDriveBackend::DiffDriveBackend(std::string p_left, std::string p_right, DriveGeometry p_geometry)
    : m_left_name(std::move(p_left)), m_right_name(std::move(p_right)), m_geometry(p_geometry)
{
}

void DiffDriveBackend::attach(robotik::Robot& p_robot)
{
    m_left = p_robot.joints().require(m_left_name);
    m_right = p_robot.joints().require(m_right_name);
}

void DiffDriveBackend::reset(robotik::Robot& p_robot)
{
    m_truth = m_start;
    m_speeds[0] = 0.0;
    m_speeds[1] = 0.0;
    robotik::JointSet& joints = p_robot.joints();
    joints.measure(m_left, joints.position(m_left), 0.0);
    joints.measure(m_right, joints.position(m_right), 0.0);
}

void DiffDriveBackend::step(robotik::Robot& p_robot, Seconds p_dt)
{
    robotik::JointSet& joints = p_robot.joints();
    double const dt = p_dt.value();
    double const blend = 1.0 - std::exp(-dt / WHEEL_TIME_CONSTANT_S);
    robotik::JointId const ids[2] = { m_left, m_right };
    for (int i = 0; i < 2; ++i)
    {
        // A disabled or position controlled wheel brakes.
        double const target =
            joints.mode(ids[i]) == robotik::JointMode::Velocity ? joints.target(ids[i]) : 0.0;
        m_speeds[i] += (target - m_speeds[i]) * blend;
        joints.measure(ids[i], joints.position(ids[i]) + m_speeds[i] * dt, m_speeds[i]);
    }
    double speed = 0.0;
    double yaw_rate = 0.0;
    m_geometry.twist(m_speeds[0], m_speeds[1], speed, yaw_rate);
    m_truth = m_geometry.integrate(m_truth, speed, yaw_rate, dt);
}

robotik::Pose DiffDriveBackend::pose() const
{
    return { { m_truth.x, m_truth.y, m_geometry.height },
             robotik::Quaternion::axisAngle({ 0.0, 0.0, 1.0 }, m_truth.yaw) };
}
