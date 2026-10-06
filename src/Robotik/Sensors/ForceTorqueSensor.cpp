// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "Robotik/Sensors/ForceTorqueSensor.hpp"

#include "Robotik/Robot/Robot.hpp"

namespace robotik
{

ForceTorqueSensor::ForceTorqueSensor(std::string p_name,
                                     ForceTorqueConfig p_config)
    : Sensor(std::move(p_name), p_config.frequency),
      m_config(std::move(p_config))
{
}

bool ForceTorqueSensor::sample(Robot const& p_robot, Seconds p_now)
{
    RobotBackend const* backend = p_robot.backend();
    if (backend == nullptr)
    {
        return false;
    }
    std::optional<Wrench> const wrench = backend->wrench(m_config.link);
    if (!wrench)
    {
        return false;
    }

    m_reading.wrench = *wrench;
    m_reading.stamp = p_now;
    auto noisy = [this](Vector3 const& p_v, double p_sigma)
    {
        return p_sigma > 0.0 ? Vector3(m_random.normal(p_v.x, p_sigma),
                                       m_random.normal(p_v.y, p_sigma),
                                       m_random.normal(p_v.z, p_sigma))
                             : p_v;
    };
    m_reading.wrench.force = noisy(m_reading.wrench.force, m_config.force_noise);
    m_reading.wrench.torque =
        noisy(m_reading.wrench.torque, m_config.torque_noise);

    if (compages::world::Entity link = p_robot.link(m_config.link))
    {
        link.set(m_reading);
    }
    return true;
}

} // namespace robotik
