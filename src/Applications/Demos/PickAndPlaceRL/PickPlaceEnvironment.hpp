// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#pragma once

#include "Robotik/Environment/Environment.hpp"
#include "Robotik/Math/Pose.hpp"
#include "Robotik/Runtime/Simulation.hpp"

#include "Compages/World/World.hpp"

#include <filesystem>
#include <memory>

namespace robotik
{
class JointGroup;
class VacuumGripper;
} // namespace robotik

// Pick and place as a reinforcement learning task, on the headless
// simulation of a scenario (its behavior tree is not used).
//
// Action (4): suction cup displacement along X, Y, Z in [-1, 1] (times
// STEP_M per step), suction on when the 4th value is positive.
// Observation (16): tip xyz, cube xyz, box xyz, cube - tip, holding, suction,
// elapsed fraction of the episode, cube in the box.
class PickPlaceEnvironment final: public robotik::Environment
{
public:

    static constexpr std::size_t OBSERVATIONS = 16;
    static constexpr std::size_t ACTIONS = 4;

    // @p_spread widens the random placement of the cube (m, each axis).
    PickPlaceEnvironment(std::filesystem::path const& p_scenario, double p_spread, std::uint32_t p_max_steps);
    ~PickPlaceEnvironment() override;

    [[nodiscard]] std::size_t observationSize() const override
    {
        return OBSERVATIONS;
    }

    [[nodiscard]] std::size_t actionSize() const override
    {
        return ACTIONS;
    }

    void reset(robotik::Seed p_seed, std::span<float> p_observation) override;
    robotik::StepResult step(std::span<float const> p_action, std::span<float> p_observation) override;

private:

    void observe(std::span<float> p_observation) const;
    [[nodiscard]] bool delivered() const;

private:

    compages::world::World m_world;
    std::unique_ptr<robotik::Simulation> m_simulation;
    robotik::JointGroup* m_arm = nullptr;
    robotik::VacuumGripper* m_gripper = nullptr;
    std::uint32_t m_max_steps;
    std::uint32_t m_steps = 0;
    bool m_grasped = false;
};

// Scripted expert reading only the observation: above the cube, down,
// suction, up, above the box, down, release.
void expertPolicy(std::span<float const> p_observation, std::span<float> p_action);
