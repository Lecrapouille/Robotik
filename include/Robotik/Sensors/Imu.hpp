// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

//! @file Imu.hpp
//! @brief Inertial measurement unit: specific force, angular velocity and
//! attitude.
#pragma once

#include "Robotik/Math/Random.hpp"
#include "Robotik/Sensors/Measurements.hpp"
#include "Robotik/Sensors/Sensor.hpp"

#include <string>

namespace robotik
{

// ****************************************************************************
//! @brief Where the IMU is and how noisy it is.
// ****************************************************************************
struct ImuConfig
{
    //!< Link carrying the IMU; empty for the robot base.
    std::string parent;
    //!< Sensor frame in the parent link frame.
    Pose mount;
    //!< Sampling rate in Hz.
    double frequency = 100.0;
    //!< Standard deviation of the accelerometer noise (SI: m/s^2).
    double accelerometer_noise = 0.0;
    //!< Standard deviation of the gyroscope noise (SI: rad/s).
    double gyroscope_noise = 0.0;
};

// ****************************************************************************
//! @brief IMU measured from the motion of its frame in the world (base pose
//! times link pose times mount), so it works with any backend that measures
//! the base. The first sample after a reset reads the robot at rest.
//!
//! @code
//! auto& imu = robot.sensors().add<robotik::Imu>(
//!     "imu", robotik::ImuConfig{ .parent = "base_link" });
//! double const heading = imu.reading().orientation.yaw().value();
//! @endcode
// ****************************************************************************
class Imu final: public Sensor
{
public:

    Imu(std::string p_name, ImuConfig p_config = {});

    [[nodiscard]] ImuConfig const& config() const
    {
        return m_config;
    }

    [[nodiscard]] ImuReading const& reading() const
    {
        return m_reading;
    }

    void seed(Seed p_seed)
    {
        m_random = Random(p_seed);
    }

protected:

    bool sample(Robot const& p_robot, Seconds p_now) override;
    void restart() override;

private:

    ImuConfig m_config;
    ImuReading m_reading;
    Random m_random;
    Pose m_previous_pose;
    Vector3 m_previous_velocity = zero3();
    Seconds m_previous_stamp{};
    int m_history = 0;
};

} // namespace robotik
