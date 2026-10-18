// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#pragma once

#include "Robotik/Math/Geometry.hpp"

#include <array>
#include <filesystem>
#include <memory>
#include <span>
#include <string>
#include <vector>

// ****************************************************************************
/// @brief What the walking policy expects, read from walk.yaml (written by
/// prepare_assets.py from the LimX deployment config). One entry per joint,
/// in the order of the policy.
// ****************************************************************************
struct WalkContract
{
    std::vector<std::string> joints;
    std::vector<double> kp;
    std::vector<double> kd;
    std::vector<double> default_angle;
    std::vector<double> action_scale;
    std::vector<double> torque_limit;
    std::vector<double> armature;
    /// Physics steps per policy step.
    int decimation = 10;
    /// Observations stacked at the policy input, newest first.
    int history = 5;
    int observations = 102;
    double clip_observations = 100.0;
    double clip_actions = 100.0;
    double scale_ang_vel = 0.25;
    double scale_dof_pos = 1.0;
    double scale_dof_vel = 0.05;
    /// Largest |vx| (m/s), |vy| (m/s), |yaw rate| (rad/s).
    std::array<double, 3> max_command{ 0.5, 0.3, 0.5 };

    /// @throws std::runtime_error on a missing or inconsistent field.
    [[nodiscard]] static WalkContract load(std::filesystem::path const& p_path);
};

// ****************************************************************************
/// @brief What the robot measures, as the policy wants it.
// ****************************************************************************
struct WalkInput
{
    /// IMU attitude in the world frame.
    robotik::Quaternion orientation;
    /// IMU angular velocity in the IMU frame (rad/s).
    robotik::Vector3 angular_velocity = robotik::zero3();
    /// Command: forward (m/s), lateral (m/s), yaw rate (rad/s).
    std::array<double, 3> command{};
    /// Joint positions and velocities, policy order.
    std::span<double const> positions;
    std::span<double const> velocities;
};

// ****************************************************************************
/// @brief LimX walking policy (ONNX, 31 joints): observation history in,
/// joint position targets out. Plain C++ over ONNX Runtime; it knows nothing
/// of the simulation.
///
/// Observation (102): angular velocity, projected gravity, command, joint
/// positions minus the default posture, joint velocities, last actions.
/// Target of joint i: action_i * action_scale_i + default_angle_i.
// ****************************************************************************
class WalkPolicy
{
public:

    /// @throws std::runtime_error if the model or its contract cannot be read.
    WalkPolicy(std::filesystem::path const& p_model, WalkContract p_contract);
    ~WalkPolicy();

    [[nodiscard]] WalkContract const& contract() const
    {
        return m_contract;
    }

    /// @brief Forgets the history and the last actions (new episode).
    void reset();

    /// @brief Runs the network once and returns one target per joint (rad).
    [[nodiscard]] std::span<double const> act(WalkInput const& p_input);

private:

    void observe(WalkInput const& p_input);

    struct Onnx;
    std::unique_ptr<Onnx> m_onnx;
    WalkContract m_contract;
    std::vector<float> m_history;
    std::vector<float> m_actions;
    std::vector<double> m_targets;
    bool m_fresh = true;
};
