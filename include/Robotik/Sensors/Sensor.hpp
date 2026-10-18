// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file Sensor.hpp
/// @brief Base class of every robot sensor: name, sampling rate and counters.
#pragma once

#include "Robotik/Math/Si.hpp"

#include <cstdint>
#include <string>

namespace robotik
{

class Robot;

// ****************************************************************************
/// @brief A named data source sampled at its own frequency.
///
/// The robot calls @ref update every step; the sensor only samples when its
/// period has elapsed. Availability is not stored here: a sensor is also a
/// resource of the robot (same name) and the robot skips the sensors whose
/// resource has failed (see @ref ResourceManager::fail).
///
/// Concrete sensors produce typed data (@ref Camera produces @ref
/// CameraFrame) and get it from a pluggable source, so the same sensor works
/// in simulation, on hardware and in RL environments. Each one has three
/// layers:
/// @li the model: its configuration (mount, rate, a @ref NoiseModel per
///     channel, latency);
/// @li the simulation: @ref sample, which measures the true value on the
///     robot and its backend, then degrades it;
/// @li the measurement: the reading, a @ref Measurement header plus typed
///     data, published once its latency has elapsed.
///
/// A sensor never knows which algorithm consumes it.
// ****************************************************************************
class Sensor
{
public:

    virtual ~Sensor() = default;

    // -------------------------------------------------------------------------
    /// @brief Sensor name, also the name of its resource.
    // -------------------------------------------------------------------------
    [[nodiscard]] std::string const& name() const
    {
        return m_name;
    }

    // -------------------------------------------------------------------------
    /// @brief Sampling frequency in Hz (0 samples at every update).
    // -------------------------------------------------------------------------
    [[nodiscard]] double frequency() const
    {
        return m_frequency;
    }

    // -------------------------------------------------------------------------
    /// @brief Number of samples produced so far.
    // -------------------------------------------------------------------------
    [[nodiscard]] std::uint64_t samples() const
    {
        return m_samples;
    }

    // -------------------------------------------------------------------------
    /// @brief Time of the last sample (negative before the first one).
    // -------------------------------------------------------------------------
    [[nodiscard]] Seconds stamp() const
    {
        return m_stamp;
    }

    // -------------------------------------------------------------------------
    /// @brief Samples the sensor if its period elapsed since the last sample,
    /// then publishes what its latency lets through.
    /// @param p_robot Robot carrying the sensor (kinematics for the mount).
    /// @param p_now Robot clock.
    // -------------------------------------------------------------------------
    void update(Robot const& p_robot, Seconds p_now)
    {
        if (p_now >= m_due && sample(p_robot, p_now))
        {
            ++m_samples;
            m_stamp = p_now;
            m_due = m_frequency > 0.0 ? p_now + Seconds(1.0 / m_frequency)
                                      : p_now;
        }
        deliver(p_now);
    }

    // -------------------------------------------------------------------------
    /// @brief The sensor resource failed: the published reading becomes
    /// invalid until the next sample.
    // -------------------------------------------------------------------------
    virtual void invalidate() {}

    // -------------------------------------------------------------------------
    /// @brief Forgets the sampling history (episode reset).
    // -------------------------------------------------------------------------
    void rewind()
    {
        m_samples = 0;
        m_stamp = Seconds(-1.0);
        m_due = Seconds(0.0);
        restart();
    }

protected:

    Sensor(std::string p_name, double p_frequency)
        : m_name(std::move(p_name)), m_frequency(p_frequency)
    {
    }

    Sensor(Sensor&&) noexcept = default;
    Sensor& operator=(Sensor&&) noexcept = default;

    // -------------------------------------------------------------------------
    /// @brief Produces one sample.
    /// @return False if no data was available (e.g. no source connected).
    // -------------------------------------------------------------------------
    virtual bool sample(Robot const& p_robot, Seconds p_now) = 0;

    /// @brief Drops the state carried between samples (see @ref rewind).
    virtual void restart() {}

    // -------------------------------------------------------------------------
    /// @brief Publishes the samples whose latency has elapsed at @p_now.
    /// Called at every update, sampled or not.
    // -------------------------------------------------------------------------
    virtual void deliver(Seconds /*p_now*/) {}

private:

    std::string m_name;
    double m_frequency = 0.0;
    std::uint64_t m_samples = 0;
    Seconds m_stamp{ -1.0 };
    Seconds m_due{ 0.0 };
};

} // namespace robotik
