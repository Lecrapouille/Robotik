// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file Imu.hpp
/// An IMU (inertial measurement unit): it measures how its link moves (the
/// acceleration felt, the turn rate) and how it is tilted.
///
/// Drones, legged robots and phones use one to keep their balance. In
/// simulation the true motion of the link is measured, then the noise model
/// spoils it.
///
/// @par Contract
/// - Promises: the reading is in the IMU frame; the first two samples of an
///   episode have no velocity or acceleration yet (finite differences).
/// - Expects: the parent link exists in the robot.
#pragma once

#include "Robotik/Common/Random.hpp"
#include "Robotik/Robot/Sensors/Measurements.hpp"
#include "Robotik/Robot/Sensors/Noise.hpp"
#include "Robotik/Robot/LinkRef.hpp"
#include "Robotik/Robot/Sensors/Sensor.hpp"

#include "Compages/World/Entity.hpp"

#include <string>

namespace robotik
{

// ****************************************************************************
/// Where the IMU is and how noisy it is.
///
/// @code
/// robotik::ImuConfig config;
/// config.parent = "torso";
/// config.frequency = 200.0;
/// config.gyroscope.sigma = 0.002; // rad/s
/// @endcode
// ****************************************************************************
struct ImuConfig
{
    /// Link carrying the IMU; empty for the robot base.
    std::string parent;
    /// Sensor frame in the parent link frame.
    Pose mount;
    /// Sampling rate in Hz.
    double frequency = 100.0;
    /// Delay between a capture and its delivery.
    Seconds latency{ 0.0 };
    /// Specific force channels (SI: m/s^2).
    NoiseModel accelerometer;
    /// Angular velocity channels (SI: rad/s).
    NoiseModel gyroscope;
};

// ****************************************************************************
/// IMU measured from the motion of its frame in the world (base pose times
/// link pose times mount), so it works with any backend that measures the
/// base. The first sample after a reset reads the robot at rest.
///
/// The velocity and the acceleration are finite differences of the last
/// poses, like a real IMU integrated over its period.
///
/// @code
/// auto& imu = robot.sensors().add<robotik::Imu>(
///     "imu", robotik::ImuConfig{ .parent = "base_link" });
/// double const heading = imu.reading().orientation.yaw().value();
/// @endcode
// ****************************************************************************
class Imu final: public Sensor
{
public:

    // -------------------------------------------------------------------------
    /// @param p_name Sensor name, also its resource name.
    /// @param p_config Mount, rate, latency and noise.
    // -------------------------------------------------------------------------
    Imu(std::string p_name, ImuConfig p_config = {});

    // -------------------------------------------------------------------------
    /// @return The configuration given at construction.
    // -------------------------------------------------------------------------
    [[nodiscard]] ImuConfig const& config() const
    {
        return m_config;
    }

    // -------------------------------------------------------------------------
    /// @return Last delivered reading. Check @c valid before use.
    // -------------------------------------------------------------------------
    [[nodiscard]] ImuReading const& reading() const
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

    /// Measures the motion of the IMU frame and adds the noise.
    bool sample(Robot const& p_robot, Seconds p_now) override;
    /// Forgets the previous poses (the next sample reads the robot at rest).
    void restart() override;
    /// Publishes the readings whose latency has elapsed.
    void deliver(Seconds p_now) override;

private:

    /// Configuration given at construction.
    ImuConfig m_config;
    /// Readings waiting for their latency, and the current one.
    LatencyBuffer<ImuReading> m_readings;
    /// Link carrying the sensor, found at the first sample of an episode;
    /// the delivered reading is attached to its entity.
    LinkRef m_link;
    /// True once @ref m_link is found.
    bool m_resolved = false;
    /// Noise generator.
    Random m_random;
    /// World pose of the IMU frame at the previous sample.
    Pose m_previous_pose;
    /// Linear velocity at the previous sample (m/s).
    Vector3 m_previous_velocity = Vector3::ZERO;
    /// Time of the previous sample.
    Seconds m_previous_stamp{};
    /// Number of previous samples known (0, 1 or 2): the finite differences
    /// need two.
    int m_history = 0;
};

} // namespace robotik
