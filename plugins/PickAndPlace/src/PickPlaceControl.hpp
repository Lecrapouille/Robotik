// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file PickPlaceControl.hpp
/// @brief Pick and place as a learning task: observation, action and reward,
/// shared by the RL plugin (one rendered episode) and Robotik-PickAndPlaceRL
/// (a pool of headless environments), plus the scripted policies.
#pragma once

#include "Robotik/Environment/SimulationEnvironment.hpp"
#include "Robotik/Math/Geometry.hpp"
#include "Robotik/Math/Random.hpp"
#include "Robotik/Robot/Robot.hpp"

#include <cstdint>
#include <span>

// Random → converged: one action (Simulator) or 8 episodes (CLI --train).
inline constexpr float PICK_PLACE_TRAIN_STEP = 1.0f / 400.0f;
inline constexpr std::uint32_t PICK_PLACE_TRAIN_EPISODES = 8;

namespace robotik
{
class JointGroup;
class VacuumGripper;
class Simulation;
} // namespace robotik

inline constexpr std::size_t PICK_PLACE_OBSERVATIONS = 16;
inline constexpr std::size_t PICK_PLACE_ACTIONS = 4;
inline constexpr char const* PICK_PLACE_CUBE = "red_cube";
inline constexpr char const* PICK_PLACE_BOX = "box";
/// An action is held for this long (ten physics steps of 0.01 s).
inline constexpr double PICK_PLACE_CONTROL_PERIOD_S = 0.1;
inline constexpr double PICK_PLACE_PHYSICS_STEP_S = 0.01;

// Episodes start with the suction cup pointing down above the table.
inline robotik::Vector3 const PICK_PLACE_READY{ 0.40, 0.0, 0.25 };
// Ry(pi): the natural wrist of the 6-axis arm.
inline robotik::Quaternion const PICK_PLACE_DOWN{ 0.0, 0.0, 1.0, 0.0 };

void readyPosture(robotik::RobotSession& p_robot,
                  robotik::VacuumGripper const& p_gripper,
                  robotik::Vector3 const& p_tip = PICK_PLACE_READY);

/// @brief The cube rests in the box and the cup let it go (ground truth).
[[nodiscard]] bool cubeInBox(robotik::Simulation const& p_simulation,
                             robotik::VacuumGripper const& p_gripper);

// ****************************************************************************
/// @brief Cup tip, believed cube and box, their offset, cup state, progress.
/// The positions are the robot's beliefs (world model, base frame). Only the
/// @c delivered flag reads the ground truth: it is marked privileged.
// ****************************************************************************
class PickPlaceObservation final: public robotik::ObservationBuilder
{
public:

    [[nodiscard]] robotik::Space space() const override;
    void setup(robotik::Simulation& p_simulation) override;
    /// @brief The episode starts with one look at the scene (oracle runs).
    void reset(robotik::Simulation& p_simulation) override;
    void observe(robotik::Simulation const& p_simulation,
                 robotik::EpisodeClock const& p_clock,
                 std::span<float> p_observation) override;

private:

    robotik::VacuumGripper const* m_gripper = nullptr;
};

// ****************************************************************************
/// @brief Cup displacement (in steps of 3 cm, clamped to the workspace) and
/// suction (on above zero), through inverse kinematics on the arm group.
// ****************************************************************************
class PickPlaceAction final: public robotik::ActionAdapter
{
public:

    [[nodiscard]] robotik::Space space() const override;
    /// @brief Finds the arm and the cup, and makes the ready posture the home.
    void setup(robotik::Simulation& p_simulation) override;
    void apply(std::span<float const> p_action,
               robotik::Simulation& p_simulation) override;

private:

    robotik::JointGroup* m_arm = nullptr;
    robotik::VacuumGripper* m_gripper = nullptr;
};

// ****************************************************************************
/// @brief Dense shaping: reach the cube top, carry it above the box, a bonus
/// at the first grasp; success when the cube rests in the box.
// ****************************************************************************
class PickPlaceReward final: public robotik::RewardEvaluator
{
public:

    void setup(robotik::Simulation const& p_simulation) override;
    void reset(robotik::Simulation const& p_simulation) override;
    [[nodiscard]] robotik::StepResult
    evaluate(robotik::Simulation const& p_simulation) override;

private:

    robotik::VacuumGripper const* m_gripper = nullptr;
    bool m_grasped = false;
};

/// @brief The three parts, ready for a @ref robotik::SimulationEnvironment.
[[nodiscard]] robotik::SimulationEnvironment::Parts pickPlaceParts();

void convergedPolicy(std::span<float const> p_observation,
                     std::span<float> p_action);

/// @brief @p_mix 0 = noise, 1 = converged recipe. In between: blend.
void pickPlacePolicy(std::span<float const> p_observation,
                     std::span<float> p_action,
                     float p_mix,
                     robotik::Random* p_noise);

/// @brief Visible phase (approach / grasp / carry / drop).
[[nodiscard]] char const* policyPhase(std::span<float const> p_observation);
