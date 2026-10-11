// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#pragma once

#include "Robotik/Environment/Environment.hpp"
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

// Episodes start with the suction cup pointing down above the table.
inline robotik::Vector3 const PICK_PLACE_READY{ 0.40, 0.0, 0.25 };
// Ry(pi): the natural wrist of the 6-axis arm.
inline robotik::Quaternion const PICK_PLACE_DOWN{ 0.0, 0.0, 1.0, 0.0 };

void readyPosture(robotik::RobotSession& p_robot,
                  robotik::VacuumGripper const& p_gripper,
                  robotik::Vector3 const& p_tip = PICK_PLACE_READY);

void applyPickPlaceAction(robotik::Simulation& p_simulation,
                          robotik::JointGroup& p_arm,
                          robotik::VacuumGripper& p_gripper,
                          std::span<float const> p_action);

void writePickPlaceObservation(robotik::Simulation const& p_simulation,
                               robotik::VacuumGripper const& p_gripper,
                               std::uint32_t p_steps,
                               std::uint32_t p_max_steps,
                               std::span<float> p_observation);

[[nodiscard]] bool cubeInBox(robotik::Simulation const& p_simulation,
                             robotik::VacuumGripper const& p_gripper);

void convergedPolicy(std::span<float const> p_observation,
                     std::span<float> p_action);

//! @brief @p_mix 0 = noise, 1 = converged recipe. In between: blend.
void pickPlacePolicy(std::span<float const> p_observation,
                     std::span<float> p_action,
                     float p_mix,
                     robotik::Random* p_noise);

//! @brief Visible phase (approach / grasp / carry / drop).
[[nodiscard]] char const* policyPhase(std::span<float const> p_observation);
