// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file RangeScanner.hpp
/// Planar range scanner (2D lidar).
#pragma once

#include "Robotik/Math/Random.hpp"
#include "Robotik/Sensors/Measurements.hpp"
#include "Robotik/Sensors/Noise.hpp"
#include "Robotik/Sensors/Sensor.hpp"

#include "Compages/World/Entity.hpp"

#include <cstdint>
#include <numbers>
#include <string>

namespace robotik
{

// ****************************************************************************
/// Where the scanner is and how it samples.
///
/// @code
/// robotik::RangeScannerConfig config;
/// config.parent = "base_link";
/// config.beams = 360;
/// config.field_of_view = Radians(std::numbers::pi); // 180 deg in front
/// config.noise.sigma = 0.01;                        // 1 cm
/// @endcode
// ****************************************************************************
struct RangeScannerConfig
{
    /// Link carrying the scanner; empty for the robot base.
    std::string parent;
    /// Scan plane: the XY plane of this frame, in the parent link frame.
    Pose mount;
    /// Number of beams, spread evenly over the field of view.
    std::uint32_t beams = 180;
    /// Angular span, centered on +X.
    Radians field_of_view{ 2.0 * std::numbers::pi };
    /// Beams longer than this see nothing.
    Length max_range{ 10.0 };
    /// Scan rate in Hz.
    double frequency = 10.0;
    /// Delay between a scan and its delivery.
    Seconds latency{ 0.0 };
    /// Range channel (SI: m), applied to hits only and saturated to
    /// [0, max_range].
    NoiseModel noise;
};

// ****************************************************************************
/// 2D lidar casting its beams through the backend
/// (@ref RobotBackend::raycast) and against the scene objects of the world.
///
/// @code
/// auto& lidar = robot.sensors().add<robotik::RangeScanner>(
///     "lidar", robotik::RangeScannerConfig{ .parent = "base_link" });
/// float const front = lidar.scan().ranges[lidar.scan().ranges.size() / 2];
/// @endcode
// ****************************************************************************
class RangeScanner final: public Sensor
{
public:

    // -------------------------------------------------------------------------
    /// @param p_name Sensor name, also its resource name.
    /// @param p_config Mount, beams, range, rate and noise.
    // -------------------------------------------------------------------------
    RangeScanner(std::string p_name, RangeScannerConfig p_config = {});

    // -------------------------------------------------------------------------
    /// @return The configuration given at construction.
    // -------------------------------------------------------------------------
    [[nodiscard]] RangeScannerConfig const& config() const
    {
        return m_config;
    }

    // -------------------------------------------------------------------------
    /// @return Last delivered scan. Check @c valid before use.
    // -------------------------------------------------------------------------
    [[nodiscard]] RangeScan const& scan() const
    {
        return m_scans.current();
    }

    // -------------------------------------------------------------------------
    /// Marks the current scan invalid (the sensor resource failed).
    // -------------------------------------------------------------------------
    void invalidate() override
    {
        m_scans.current().valid = false;
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

    /// Casts every beam and adds the noise.
    bool sample(Robot const& p_robot, Seconds p_now) override;
    /// Clears the scans waiting for their latency.
    void restart() override;
    /// Publishes the scans whose latency has elapsed.
    void deliver(Seconds p_now) override;

private:

    /// Configuration given at construction.
    RangeScannerConfig m_config;
    /// Scans waiting for their latency, and the current one.
    LatencyBuffer<RangeScan> m_scans;
    /// Link entity the delivered scan is attached to.
    compages::world::Entity m_holder;
    /// Noise generator.
    Random m_random;
};

} // namespace robotik
