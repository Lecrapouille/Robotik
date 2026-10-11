// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

//! @file ForceTorqueSensor.hpp
//! @brief Six-axis force/torque sensor (wrist, ankle, gripper).
#pragma once

#include "Robotik/Math/Random.hpp"
#include "Robotik/Sensors/Measurements.hpp"
#include "Robotik/Sensors/Noise.hpp"
#include "Robotik/Sensors/Sensor.hpp"

#include "Compages/World/Entity.hpp"

#include <string>

namespace robotik
{

// ****************************************************************************
//! @brief Which link is measured and how noisy the measure is.
// ****************************************************************************
struct ForceTorqueConfig
{
    //!< Link whose wrench from its parent link is measured.
    std::string link;
    //!< Sampling rate in Hz.
    double frequency = 500.0;
    //!< Delay between a capture and its delivery.
    Seconds latency{ 0.0 };
    //!< Force channels (SI: N).
    NoiseModel force;
    //!< Torque channels (SI: N.m).
    NoiseModel torque;
};

// ****************************************************************************
//! @brief Wrench measured through the backend (@ref RobotBackend::wrench).
//! A backend that cannot measure wrenches leaves the sensor without samples.
// ****************************************************************************
class ForceTorqueSensor final: public Sensor
{
public:

    ForceTorqueSensor(std::string p_name, ForceTorqueConfig p_config);

    [[nodiscard]] ForceTorqueConfig const& config() const
    {
        return m_config;
    }

    //! @brief Last delivered reading, in the frame of the measured link.
    [[nodiscard]] ForceTorqueReading const& reading() const
    {
        return m_readings.current();
    }

    void invalidate() override
    {
        m_readings.current().valid = false;
    }

    void seed(Seed p_seed)
    {
        m_random = Random(p_seed);
    }

protected:

    bool sample(Robot const& p_robot, Seconds p_now) override;
    void restart() override;
    void deliver(Seconds p_now) override;

private:

    ForceTorqueConfig m_config;
    LatencyBuffer<ForceTorqueReading> m_readings;
    //!< Link entity the delivered reading is attached to.
    compages::world::Entity m_holder;
    Random m_random;
};

} // namespace robotik
