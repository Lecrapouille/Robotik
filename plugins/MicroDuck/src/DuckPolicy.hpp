// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#pragma once

#include "Robotik/Common/Math.hpp"

#include <array>
#include <filesystem>
#include <memory>
#include <span>
#include <string>
#include <vector>

// ****************************************************************************
/// @brief What the walking policy expects, read from walk.yaml (written by
/// prepare_assets.py). One entry per servo, in the order of the policy.
/// The mouth is not in this model: the training body has 14 servos.
// ****************************************************************************
struct DuckContract
{
    std::vector<std::string> joints;
    std::vector<double> kp;
    /// Derivative gain of the position actuator. Zero: viscosity is @ref
    /// damping.
    std::vector<double> kd;
    /// Viscous joint damping (N·m·s/rad), applied by MuJoCo, not by the PD law.
    std::vector<double> damping;
    std::vector<double> default_angle;
    std::vector<double> action_scale;
    std::vector<double> torque_limit;
    std::vector<double> armature;
    /// Physics steps per policy step.
    int decimation = 4;
    /// Physics step (s). The policy runs at 1 / (decimation * physics_step).
    double physics_step = 0.005;
    /// Largest |vx| (m/s), |vy| (m/s), |yaw rate| (rad/s).
    std::array<double, 3> max_command{ 0.4, 0.3, 1.0 };

    /// @throws std::runtime_error on a missing or inconsistent field.
    [[nodiscard]] static DuckContract load(std::filesystem::path const& p_path);
};

// ****************************************************************************
/// @brief What the robot measures, as the policy wants it.
// ****************************************************************************
struct DuckInput
{
    /// IMU attitude in the world frame.
    robotik::Quaternion orientation;
    /// IMU angular velocity in the trunk frame (rad/s).
    robotik::Vector3 angular_velocity = robotik::Vector3::ZERO;
    /// Command: forward (m/s), lateral (m/s), yaw rate (rad/s).
    std::array<double, 3> command{};
    /// Joint positions and velocities, policy order.
    std::span<double const> positions;
    std::span<double const> velocities;
};

// ****************************************************************************
/// @brief Microduck walking policy (ONNX, 14 servos): one observation in,
/// joint position offsets out. Plain C++ over ONNX Runtime; it knows nothing
/// of the simulation.
///
/// Observation (61), the order of duck-control obs.rs:
/// gyro (3), projected gravity (3), joint position minus home (14), joint
/// velocity (14), previous action (14), then the command block twist (3),
/// head (4, zeros) and body (6, zeros: x, y, z, roll, pitch, yaw).
/// Target of joint i: action_i * action_scale_i + default_angle_i.
// ****************************************************************************
class DuckPolicy
{
public:

    /// @throws std::runtime_error if the model or its contract cannot be read.
    DuckPolicy(std::filesystem::path const& p_model, DuckContract p_contract);
    ~DuckPolicy();

    [[nodiscard]] DuckContract const& contract() const
    {
        return m_contract;
    }

    /// @brief Forgets the previous action (new episode).
    void reset();

    /// @brief Runs the network once and returns one target per joint (rad).
    [[nodiscard]] std::span<double const> act(DuckInput const& p_input);

private:

    struct Onnx;
    std::unique_ptr<Onnx> m_onnx;
    DuckContract m_contract;
    std::vector<float> m_observation;
    std::vector<float> m_actions;
    std::vector<double> m_targets;
};
