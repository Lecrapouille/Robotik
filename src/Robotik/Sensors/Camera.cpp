// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "Robotik/Sensors/Camera.hpp"

#include "Robotik/Backends/PinocchioBackend.hpp"
#include "Robotik/Robot/Robot.hpp"
#include "Robotik/Sensors/Measurements.hpp"

#include <algorithm>
#include <cmath>

namespace robotik
{

CameraIntrinsics CameraIntrinsics::fromFov(std::uint32_t p_width,
                                           std::uint32_t p_height,
                                           Radians p_fov)
{
    CameraIntrinsics intrinsics;
    intrinsics.width = p_width;
    intrinsics.height = p_height;
    intrinsics.fy = 0.5 * p_height / std::tan(0.5 * p_fov.value());
    intrinsics.fx = intrinsics.fy;
    intrinsics.cx = 0.5 * p_width;
    intrinsics.cy = 0.5 * p_height;
    return intrinsics;
}

Radians CameraIntrinsics::fov() const
{
    return Radians(2.0 * std::atan(0.5 * height / fy));
}

Vector3 CameraIntrinsics::ray(double p_u, double p_v) const
{
    return compages::core::vector::normalize(
        Vector3((p_u - cx) / fx, (p_v - cy) / fy, 1.0));
}

std::optional<std::array<double, 2>>
CameraIntrinsics::project(Vector3 const& p_point) const
{
    if (p_point.z <= 1e-9)
    {
        return std::nullopt;
    }
    return std::array<double, 2>{ cx + fx * p_point.x / p_point.z,
                                  cy + fy * p_point.y / p_point.z };
}

Camera::Camera(std::string p_name, CameraConfig p_config)
    : Sensor(std::move(p_name), p_config.frequency),
      m_config(std::move(p_config))
{
}

bool Camera::sample(Robot const& p_robot, Seconds p_now)
{
    if (m_source == nullptr)
    {
        return false;
    }

    m_frame.intrinsics = m_config.intrinsics;
    m_frame.pose =
        m_config.parent.empty()
            ? m_config.mount
            : p_robot.kinematics().framePose(m_config.parent) * m_config.mount;
    m_frame.stamp = p_now;
    if (!m_source->capture(*this, m_frame))
    {
        return false;
    }
    ++m_frame.sequence;

    if (m_config.noise > 0.0 && m_frame.rgb.format() != PixelFormat::Depth32F)
    {
        double const sigma = m_config.noise * 255.0;
        for (std::uint8_t& channel : m_frame.rgb.bytes())
        {
            double const noisy = m_random.normal(channel, sigma);
            channel = static_cast<std::uint8_t>(std::clamp(noisy, 0.0, 255.0));
        }
    }

    for (Callback const& callback : m_callbacks)
    {
        callback(m_frame);
    }

    compages::world::Entity holder =
        m_config.parent.empty() ? p_robot.root()
                                : p_robot.link(m_config.parent);
    if (holder)
    {
        holder.set(CameraReading{ m_frame.pose,
                                  m_frame.stamp,
                                  m_frame.sequence,
                                  m_frame.intrinsics.width,
                                  m_frame.intrinsics.height });
    }
    return true;
}

} // namespace robotik
