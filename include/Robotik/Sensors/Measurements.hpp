// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file Measurements.hpp
/// @brief Physical quantities measured on a robot, in SI units.
///
/// These plain structs are what sensors produce and what backends report.
/// Every sensor reading starts with a @ref Measurement header: when it was
/// captured, in which frame its data are expressed, its rank and whether it
/// is valid. Sensors also attach their last reading to the entity of their
/// link, as an ECS component, so a viewer or a logger can query the world
/// without knowing the sensors:
/// @code
/// world.each<robotik::ImuReading>([](auto p_link, auto const& p_imu) { ... });
/// @endcode
#pragma once

#include "Robotik/Math/Geometry.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace robotik
{

// ****************************************************************************
/// @brief Header shared by every sensor reading.
// ****************************************************************************
struct Measurement
{
    /// Capture time on the robot clock. With latency, the reading is
    /// delivered later but keeps this stamp.
    Seconds stamp{};
    /// Frame the data are expressed in: the sensor frame, named after the
    /// sensor, unless the reading says otherwise.
    std::string frame;
    /// Samples captured since the last reset, this one included.
    std::uint64_t sequence = 0;
    /// False before the first sample and while the sensor is unavailable.
    bool valid = false;
};

// ****************************************************************************
/// @brief Velocity of a frame.
// ****************************************************************************
struct Twist
{
    /// Linear velocity (SI: m/s).
    Vector3 linear = zero3();
    /// Angular velocity (SI: rad/s).
    Vector3 angular = zero3();
};

// ****************************************************************************
/// @brief Force and torque applied at the origin of a frame.
// ****************************************************************************
struct Wrench
{
    /// Force (SI: N).
    Vector3 force = zero3();
    /// Torque about the frame origin (SI: N.m).
    Vector3 torque = zero3();
};

// ****************************************************************************
/// @brief Pose and velocity of the robot base in the world frame.
///
/// Ground truth in simulation, identity for a robot bolted to the world. A
/// mobile robot does not know it: it estimates it (odometry, localization).
// ****************************************************************************
struct BaseState
{
    Pose pose;
    /// Expressed in the world frame.
    Twist twist;
};

// ****************************************************************************
/// @brief One inertial measurement, in the sensor frame.
// ****************************************************************************
struct ImuReading : Measurement
{
    /// Specific force: acceleration minus gravity, so +9.81 along the up
    /// axis at rest (SI: m/s^2), in the sensor frame.
    Vector3 acceleration = zero3();
    /// Angular velocity (SI: rad/s), in the sensor frame.
    Vector3 angular_velocity = zero3();
    /// Sensor frame in the world frame (fused attitude).
    Quaternion orientation;
};

// ****************************************************************************
/// @brief One sweep of a planar range scanner.
// ****************************************************************************
struct RangeScan : Measurement
{
    /// One distance per beam (SI: m); max_range when nothing was hit.
    std::vector<float> ranges;
    /// Beam angles in the scan plane (XY of the sensor frame).
    Radians angle_min{};
    Radians increment{};
    Length max_range{};
};

// ****************************************************************************
/// @brief Wrench a link receives from its parent link.
// ****************************************************************************
struct ForceTorqueReading : Measurement
{
    /// In the frame of the measured link.
    Wrench wrench;
};

// ****************************************************************************
/// @brief What a camera publishes on its link entity (no pixels).
// ****************************************************************************
struct CameraReading : Measurement
{
    /// Optical frame in the robot base frame.
    Pose pose;
    std::uint32_t width = 0;
    std::uint32_t height = 0;
};

} // namespace robotik
