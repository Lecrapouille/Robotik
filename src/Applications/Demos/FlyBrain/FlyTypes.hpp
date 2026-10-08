// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

// @file FlyTypes.hpp
// @brief Observation, action, posture and scenario of the fly demo.
//
// Metres, radians and seconds of the x100 model. The brain only sees
// @ref FlyObservation and @ref FlyAction.
#pragma once

#include "Robotik/Math/Geometry.hpp"

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <span>
#include <string>
#include <vector>

// ****************************************************************************
//! @brief What the fly brain may read.
//!
//! Target bearing is the food beacon: positive means the target is to the
//! left (+y).
// ****************************************************************************
struct FlyObservation
{
    //!< Left eye, 1 when an obstacle touches the eye, 0 at the end of the ray.
    float visual_left = 0.0f;
    //!< Centre eye, same scale as @ref visual_left.
    float visual_center = 0.0f;
    //!< Right eye, same scale as @ref visual_left.
    float visual_right = 0.0f;
    //!< Yaw rate of the thorax (SI: rad/s). Positive yaws left.
    float angular_velocity = 0.0f;
    //!< Speed along the heading (SI: m/s).
    float forward_velocity = 0.0f;
    //!< Thorax height (SI: m).
    float altitude = 0.0f;
    //!< Range of the nearest obstacle along the three rays (SI: m).
    float distance_to_obstacle = 0.0f;
    //!< Food direction relative to the heading (SI: rad). Positive is left.
    float target_bearing = 0.0f;
    //!< Horizontal distance from the thorax to the food (SI: m).
    float target_distance = 0.0f;

    //!< Number of floats @ref from and @ref write exchange.
    static constexpr std::size_t SIZE = 9;

    // ------------------------------------------------------------------------
    //! @brief Reads the nine fields above, in the order they are declared.
    // ------------------------------------------------------------------------
    [[nodiscard]] static FlyObservation from(std::span<float const> p_values);

    // ------------------------------------------------------------------------
    //! @brief Writes the nine fields above, in the order they are declared.
    // ------------------------------------------------------------------------
    void write(std::span<float> p_values) const;
};

// ****************************************************************************
//! @brief What the fly brain decides.
//!
//! The controller maps this onto the body. The brain does not name joints.
// ****************************************************************************
struct FlyAction
{
    //!< Forward command in [-1, 1]. Positive goes along the heading.
    float forward = 0.0f;
    //!< Yaw command in [-1, 1]. Positive yaws left.
    float turn = 0.0f;
    //!< Vertical command in [-1, 1]. Positive climbs. Zero holds altitude.
    float lift = 0.0f;

    //!< Number of floats @ref from and @ref write exchange.
    static constexpr std::size_t SIZE = 3;

    // ------------------------------------------------------------------------
    //! @brief Reads forward, turn, then lift.
    // ------------------------------------------------------------------------
    [[nodiscard]] static FlyAction from(std::span<float const> p_values);

    // ------------------------------------------------------------------------
    //! @brief Writes forward, turn, then lift.
    // ------------------------------------------------------------------------
    void write(std::span<float> p_values) const;
};

// ****************************************************************************
//! @brief Joint targets for the URDF.
//!
//! Left and right share one value: the model is mirrored, so equal commands
//! move both sides together.
// ****************************************************************************
struct FlyPosture
{
    //!< Wing sweep (SI: rad). Zero holds the wing out to the side.
    double wing_sweep = 0.0;
    //!< Wing deviation (SI: rad).
    double wing_deviation = 0.0;
    //!< Wing pitch (SI: rad).
    double wing_pitch = 0.0;
    //!< Haltere flap (SI: rad), both sides.
    double haltere = 0.0;
    //!< Head yaw in the thorax (SI: rad). Positive yaws the head left.
    double neck_yaw = 0.0;
    //!< Head pitch (SI: rad).
    double neck_pitch = 0.0;
    //!< Abdomen pitch (SI: rad).
    double abdomen_pitch = 0.0;
    //!< Coxa pitch of every leg (SI: rad).
    double coxa_pitch = 0.0;
    //!< Femur pitch of every leg (SI: rad).
    double femur_pitch = 0.0;
    //!< Tibia pitch of every leg (SI: rad).
    double tibia_pitch = 0.0;
    //!< Tarsus pitch of every leg (SI: rad).
    double tarsus_pitch = 0.0;
};

// ****************************************************************************
//! @brief Axis-aligned obstacle. Position is the centre, size the full extents.
//!
//! x forward, y left, z up, in metres.
// ****************************************************************************
struct FlyBox
{
    //!< Centre of the box (SI: m).
    robotik::Vector3 position;
    //!< Full width, depth and height (SI: m).
    robotik::Vector3 size;
};

// ****************************************************************************
//! @brief Kinematic state of the thorax.
//!
//! x forward, y left, z up, in metres. Yaw is about z; positive yaws left.
// ****************************************************************************
struct FlyPlant
{
    //!< Thorax position along the arena (SI: m).
    double x = 0.0;
    //!< Thorax position, left of the arena centre (SI: m).
    double y = 0.0;
    //!< Thorax height (SI: m).
    double z = 1.0;
    //!< Heading about z (SI: rad).
    double yaw = 0.0;
    //!< Speed along the heading (SI: m/s).
    double speed = 0.0;
    //!< Yaw rate (SI: rad/s).
    double yaw_rate = 0.0;
    //!< Climb rate (SI: m/s).
    double vertical_speed = 0.0;
};

// ****************************************************************************
//! @brief One episode description, read from a YAML file.
// ****************************************************************************
struct FlyScenario
{
    //!< Scenario name.
    std::string name = "fly_obstacle_avoidance";
    //!< Master seed: obstacle jitter, sensor noise and Poisson draws.
    std::uint64_t seed = 123456;
    //!< Environment step (SI: s).
    double dt = 0.01;
    //!< Episode length (SI: s). The step limit is horizon / dt.
    double horizon = 40.0;
    //!< Arena extents along x, y and z (SI: m).
    robotik::Vector3 arena{ 10.0, 10.0, 5.0 };
    //!< Thorax at the start of an episode, before jitter.
    FlyPlant fly{};
    //!< Obstacles before the per-episode jitter.
    std::vector<FlyBox> obstacles;
    //!< Food position (SI: m).
    robotik::Vector3 target{ 8.0, 0.0, 1.0 };
    //!< Standard deviation of the gaussian noise added to each eye.
    double sensor_noise = 0.01;
    //!< Half-width of the uniform jitter applied to each obstacle (SI: m).
    double layout_jitter = 0.02;
    //!< Agent name recorded in the file. The demo does not branch on it.
    std::string agent = "fly_brain";
    //!< URDF path, resolved against the scenario directory.
    std::filesystem::path robot_model;

    // ------------------------------------------------------------------------
    //! @brief Parses @p_path. Paths inside the file are relative to its
    //! directory.
    // ------------------------------------------------------------------------
    [[nodiscard]] static FlyScenario load(std::filesystem::path const& p_path);
};

//!< URDF prefixes of the six legs, fore to hind, left then right.
inline constexpr char const* FLY_LEG_NAMES[] = { "LF", "LM", "LH", "RF", "RM", "RH" };
