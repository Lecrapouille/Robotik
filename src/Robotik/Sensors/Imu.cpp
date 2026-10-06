// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "Robotik/Sensors/Imu.hpp"

#include "Robotik/Robot/Robot.hpp"

#include <algorithm>
#include <cmath>

#define STANDARD_GRAVITY 9.80665

namespace robotik
{

namespace
{

//! Rotation vector (axis * angle) of a unit quaternion.
Vector3 rotationVector(Quaternion p_q)
{
    if (p_q.w < 0.0)
    {
        p_q = Quaternion(-p_q.w, -p_q.x, -p_q.y, -p_q.z);
    }
    Vector3 const axis(p_q.x, p_q.y, p_q.z);
    double const sine = compages::core::vector::norm(axis);
    if (sine < 1e-12)
    {
        return axis * 2.0;
    }
    double const angle = 2.0 * std::atan2(sine, std::clamp(p_q.w, -1.0, 1.0));
    return axis * (angle / sine);
}

} // namespace

Imu::Imu(std::string p_name, ImuConfig p_config)
    : Sensor(std::move(p_name), p_config.frequency),
      m_config(std::move(p_config))
{
}

void Imu::restart()
{
    m_history = 0;
    m_previous_velocity = zero3();
}

bool Imu::sample(Robot const& p_robot, Seconds p_now)
{
    Pose const pose = p_robot.worldPose(m_config.parent) * m_config.mount;
    Vector3 velocity = zero3();
    Vector3 acceleration = zero3();
    Vector3 angular_velocity = zero3();

    double const dt = (p_now - m_previous_stamp).value();
    if (m_history > 0 && dt > 0.0)
    {
        velocity = (pose.position - m_previous_pose.position) * (1.0 / dt);
        if (m_history > 1)
        {
            acceleration = (velocity - m_previous_velocity) * (1.0 / dt);
        }
        angular_velocity =
            rotationVector(m_previous_pose.rotation.conjugate() *
                           pose.rotation) *
            (1.0 / dt);
    }

    Quaternion const to_sensor = pose.rotation.conjugate();
    Vector3 const specific_force =
        to_sensor * (acceleration + Vector3(0.0, 0.0, STANDARD_GRAVITY));

    m_reading.acceleration = specific_force;
    m_reading.angular_velocity = angular_velocity;
    m_reading.orientation = pose.rotation;
    m_reading.stamp = p_now;
    if (m_config.accelerometer_noise > 0.0)
    {
        double const sigma = m_config.accelerometer_noise;
        m_reading.acceleration = m_reading.acceleration +
                                 Vector3(m_random.normal(0.0, sigma),
                                         m_random.normal(0.0, sigma),
                                         m_random.normal(0.0, sigma));
    }
    if (m_config.gyroscope_noise > 0.0)
    {
        double const sigma = m_config.gyroscope_noise;
        m_reading.angular_velocity = m_reading.angular_velocity +
                                     Vector3(m_random.normal(0.0, sigma),
                                             m_random.normal(0.0, sigma),
                                             m_random.normal(0.0, sigma));
    }

    m_previous_pose = pose;
    m_previous_velocity = velocity;
    m_previous_stamp = p_now;
    m_history = std::min(m_history + 1, 2);

    compages::world::Entity holder =
        m_config.parent.empty() ? p_robot.root() : p_robot.link(m_config.parent);
    if (holder)
    {
        holder.set(m_reading);
    }
    return true;
}

} // namespace robotik
