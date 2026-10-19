// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file ForceTorqueSensor.hpp
/// Six-axis force/torque sensor (wrist, ankle, gripper).
#pragma once

#include "Robotik/Common/Random.hpp"
#include "Robotik/Sensors/Measurements.hpp"
#include "Robotik/Sensors/Noise.hpp"
#include "Robotik/Sensors/Sensor.hpp"

#include "Compages/World/Entity.hpp"

#include <string>

namespace robotik
{

// ****************************************************************************
/// Which link is measured and how noisy the measure is.
///
/// @code
/// robotik::ForceTorqueConfig config;
/// config.link = "wrist_link";
/// config.frequency = 1000.0;
/// config.force.sigma = 0.1; // N
/// @endcode
// ****************************************************************************
struct ForceTorqueConfig
{
    /// Link whose wrench from its parent link is measured.
    std::string link;
    /// Sampling rate in Hz.
    double frequency = 500.0;
    /// Delay between a capture and its delivery.
    Seconds latency{ 0.0 };
    /// Force channels (SI: N).
    NoiseModel force;
    /// Torque channels (SI: N.m).
    NoiseModel torque;
};

// ****************************************************************************
/// Wrench measured through the backend (@ref RobotBackend::wrench). A backend
/// that cannot measure wrenches leaves the sensor without samples.
///
/// Useful to feel a contact: a gripper that touches a table, a foot that hits
/// the ground.
///
/// @code
/// auto& ft = robot.sensors().add<robotik::ForceTorqueSensor>("wrist_ft",
/// config);
/// // later, in a skill:
/// if (ft.reading().valid && ft.reading().wrench.force.z > 5.0)
///     return robotik::Status::SUCCESS; // touched
/// @endcode
// ****************************************************************************
class ForceTorqueSensor final: public Sensor
{
public:

    // -------------------------------------------------------------------------
    /// @param p_name Sensor name, also its resource name.
    /// @param p_config Measured link, rate, latency and noise.
    // -------------------------------------------------------------------------
    ForceTorqueSensor(std::string p_name, ForceTorqueConfig p_config);

    // -------------------------------------------------------------------------
    /// @return The configuration given at construction.
    // -------------------------------------------------------------------------
    [[nodiscard]] ForceTorqueConfig const& config() const
    {
        return m_config;
    }

    // -------------------------------------------------------------------------
    /// @return Last delivered reading, in the frame of the measured link.
    ///   Check @c valid before use.
    // -------------------------------------------------------------------------
    [[nodiscard]] ForceTorqueReading const& reading() const
    {
        return m_readings.current();
    }

    // -------------------------------------------------------------------------
    /// Marks the current reading invalid (the sensor resource failed).
    // -------------------------------------------------------------------------
    void invalidate() override
    {
        m_readings.current().valid = false;
    }

    // -------------------------------------------------------------------------
    /// Restarts the noise generator.
    /// @param p_seed Same seed, same noise.
    // -------------------------------------------------------------------------
    void seed(Seed p_seed)
    {
        m_random = Random(p_seed);
    }

protected:

    /// Reads the wrench from the backend and adds the noise.
    bool sample(Robot const& p_robot, Seconds p_now) override;
    /// Clears the readings waiting for their latency.
    void restart() override;
    /// Publishes the readings whose latency has elapsed.
    void deliver(Seconds p_now) override;

private:

    /// Configuration given at construction.
    ForceTorqueConfig m_config;
    /// Readings waiting for their latency, and the current one.
    LatencyBuffer<ForceTorqueReading> m_readings;
    /// Link entity the delivered reading is attached to.
    compages::world::Entity m_holder;
    /// Noise generator.
    Random m_random;
};

} // namespace robotik
