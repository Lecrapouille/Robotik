// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "Robotik/Robot/Sensors/ForceTorqueSensor.hpp"

#include "Robotik/Robot/Robot.hpp"

namespace robotik
{

ForceTorqueSensor::ForceTorqueSensor(std::string p_name,
                                     ForceTorqueConfig p_config)
    : Sensor(std::move(p_name), p_config.frequency),
      m_config(std::move(p_config))
{
    m_readings.latency(m_config.latency);
}

void ForceTorqueSensor::restart()
{
    m_resolved = false;
    m_readings.clear();
}

void ForceTorqueSensor::deliver(Seconds p_now)
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

    ForceTorqueReading reading;
    reading.stamp = p_now;
    reading.frame = m_config.link;
    reading.sequence = samples() + 1u;
    reading.valid = true;
    reading.wrench.force = applyNoise(m_config.force, wrench->force, m_random);
    reading.wrench.torque = applyNoise(m_config.torque, wrench->torque, m_random);
    m_readings.push(p_now, std::move(reading));
    if (!m_resolved)
    {
        m_holder = p_robot.link(m_config.link);
        m_resolved = true;
    }
    return true;
}

} // namespace robotik
