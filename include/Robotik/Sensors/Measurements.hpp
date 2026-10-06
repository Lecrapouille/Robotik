// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

//! @file Measurements.hpp
//! @brief Physical quantities measured on a robot, in SI units.
//!
//! These plain structs are what sensors produce and what backends report.
//! Sensors also attach their last reading to the entity of their link, as an
//! ECS component, so a viewer or a logger can query the world without knowing
//! the sensors:
//! @code
//! world.each<robotik::ImuReading>([](auto p_link, auto const& p_imu) { ... });
//! @endcode
#pragma once

#include "Robotik/Math/Geometry.hpp"

#include <vector>

namespace robotik
{

// ****************************************************************************
//! @brief Velocity of a frame.
// ****************************************************************************
struct Twist
{
    //!< Linear velocity (SI: m/s).
    Vector3 linear = zero3();
    //!< Angular velocity (SI: rad/s).
    Vector3 angular = zero3();
};

// ****************************************************************************
//! @brief Force and torque applied at the origin of a frame.
// ****************************************************************************
struct Wrench
{
    //!< Force (SI: N).
    Vector3 force = zero3();
    //!< Torque about the frame origin (SI: N.m).
    Vector3 torque = zero3();
};

// ****************************************************************************
//! @brief Pose and velocity of the robot base in the world frame.
//!
//! Ground truth in simulation, identity for a robot bolted to the world. A
//! mobile robot does not know it: it estimates it (odometry, localization).
// ****************************************************************************
struct BaseState
{
    Pose pose;
    //!< Expressed in the world frame.
    Twist twist;
};

// ****************************************************************************
//! @brief One inertial measurement, in the sensor frame.
// ****************************************************************************
struct ImuReading
{
    //!< Specific force: acceleration minus gravity, so +9.81 along the up
    //!< axis at rest (SI: m/s^2).
    Vector3 acceleration = zero3();
    //!< Angular velocity (SI: rad/s).
    Vector3 angular_velocity = zero3();
    //!< Sensor frame in the world frame (fused attitude).
    Quaternion orientation;
    Seconds stamp{};
};

// ****************************************************************************
//! @brief One planar range scan (2D lidar), counter-clockwise about +Z of the
//! sensor, 0 rad along +X.
// ****************************************************************************
struct RangeScan
{
    //!< One distance per beam (SI: m); max_range when nothing was hit.
    std::vector<float> ranges;
    Radians angle_min{};
    Radians increment{};
    Length max_range{};
    Seconds stamp{};
};

// ****************************************************************************
//! @brief Wrench measured by a force/torque sensor, in its link frame.
// ****************************************************************************
struct ForceTorqueReading
{
    Wrench wrench;
    Seconds stamp{};
};

// ****************************************************************************
//! @brief Last camera capture, without the pixels (those stay on @ref Camera).
// ****************************************************************************
struct CameraReading
{
    Pose pose;
    Seconds stamp{};
    std::uint64_t sequence = 0;
    std::uint32_t width = 0;
    std::uint32_t height = 0;
};

} // namespace robotik
