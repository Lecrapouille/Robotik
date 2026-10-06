// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

//! @file RangeScanner.hpp
//! @brief Planar range scanner (2D lidar).
#pragma once

#include "Robotik/Math/Random.hpp"
#include "Robotik/Sensors/Measurements.hpp"
#include "Robotik/Sensors/Sensor.hpp"

#include <cstdint>
#include <numbers>
#include <string>

namespace robotik
{

// ****************************************************************************
//! @brief Where the scanner is and how it samples.
// ****************************************************************************
struct RangeScannerConfig
{
    //!< Link carrying the scanner; empty for the robot base.
    std::string parent;
    //!< Scan plane: the XY plane of this frame, in the parent link frame.
    Pose mount;
    std::uint32_t beams = 180;
    //!< Angular span, centered on +X.
    Radians field_of_view{ 2.0 * std::numbers::pi };
    Length max_range{ 10.0 };
    //!< Scan rate in Hz.
    double frequency = 10.0;
    //!< Standard deviation of the range noise (SI: m).
    double noise = 0.0;
};

// ****************************************************************************
//! @brief 2D lidar casting its beams through the backend
//! (@ref RobotBackend::raycast) and against the scene objects of the world.
//!
//! @code
//! auto& lidar = robot.sensors().add<robotik::RangeScanner>(
//!     "lidar", robotik::RangeScannerConfig{ .parent = "base_link" });
//! float const front = lidar.scan().ranges[lidar.scan().ranges.size() / 2];
//! @endcode
// ****************************************************************************
class RangeScanner final: public Sensor
{
public:

    RangeScanner(std::string p_name, RangeScannerConfig p_config = {});

    [[nodiscard]] RangeScannerConfig const& config() const
    {
        return m_config;
    }

    [[nodiscard]] RangeScan const& scan() const
    {
        return m_scan;
    }

    void seed(Seed p_seed)
    {
        m_random = Random(p_seed);
    }

protected:

    bool sample(Robot const& p_robot, Seconds p_now) override;

private:

    RangeScannerConfig m_config;
    RangeScan m_scan;
    Random m_random;
};

} // namespace robotik
