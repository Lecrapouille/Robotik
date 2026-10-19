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

/// Rotation vector (axis * angle) of a unit quaternion.
static Vector3 rotationVector(Quaternion p_q)
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

Imu::Imu(std::string p_name, ImuConfig p_config)
    : Sensor(std::move(p_name), p_config.frequency),
      m_config(std::move(p_config))
{
    m_readings.latency(m_config.latency);
}

void Imu::restart()
{
    m_history = 0;
    m_previous_velocity = Vector3::ZERO;
    m_readings.clear();
}

void Imu::deliver(Seconds p_now)
{
    if (!m_readings.advance(p_now))
    {
        return;
    }
    if (compages::world::Entity holder = m_holder)
    {
        holder.set(m_readings.current());
    }
}

bool Imu::sample(Robot const& p_robot, Seconds p_now)
{
    Pose const pose = p_robot.worldPose(m_config.parent) * m_config.mount;
    Vector3 velocity = Vector3::ZERO;
    Vector3 acceleration = Vector3::ZERO;
    Vector3 angular_velocity = Vector3::ZERO;

    double const dt = (p_now - m_previous_stamp).value();
    if (m_history > 0 && dt > 0.0)
    {
        velocity = (pose.position - m_previous_pose.position) * (1.0 / dt);
        if (m_history > 1)
        {
            acceleration = (velocity - m_previous_velocity) * (1.0 / dt);
        }
        angular_velocity = rotationVector(m_previous_pose.rotation.conjugate() *
                                          pose.rotation) *
                           (1.0 / dt);
    }

    Quaternion const to_sensor = pose.rotation.conjugate();
    Vector3 const specific_force =
        to_sensor * (acceleration + Vector3(0.0, 0.0, STANDARD_GRAVITY));

    ImuReading reading;
    reading.stamp = p_now;
    reading.frame = name();
    reading.sequence = samples() + 1u;
    reading.valid = true;
    reading.acceleration =
        m_config.accelerometer.apply(specific_force, m_random);
    reading.angular_velocity =
        m_config.gyroscope.apply(angular_velocity, m_random);
    reading.orientation = pose.rotation;
    m_readings.push(p_now, std::move(reading));

    m_previous_pose = pose;
    m_previous_velocity = velocity;
    m_previous_stamp = p_now;
    m_history = std::min(m_history + 1, 2);

    m_holder = m_config.parent.empty() ? p_robot.root()
                                       : p_robot.link(m_config.parent);
    return true;
}

} // namespace robotik
