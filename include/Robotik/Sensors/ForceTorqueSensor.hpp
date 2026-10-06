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
#include "Robotik/Sensors/Sensor.hpp"

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
    //!< Standard deviation of the force noise (SI: N).
    double force_noise = 0.0;
    //!< Standard deviation of the torque noise (SI: N.m).
    double torque_noise = 0.0;
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

    [[nodiscard]] ForceTorqueReading const& reading() const
    {
        return m_reading;
    }

    void seed(Seed p_seed)
    {
        m_random = Random(p_seed);
    }

protected:

    bool sample(Robot const& p_robot, Seconds p_now) override;

private:

    ForceTorqueConfig m_config;
    ForceTorqueReading m_reading;
    Random m_random;
};

} // namespace robotik
