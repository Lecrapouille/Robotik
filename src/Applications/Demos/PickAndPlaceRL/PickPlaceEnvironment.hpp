// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#pragma once

#include "PickPlaceControl.hpp"
#include "PickPlaceMission.hpp"

#include "Robotik/Environment/Environment.hpp"
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
class PickPlaceEnvironment final: public robotik::Environment
{
public:

    static constexpr std::size_t OBSERVATIONS = PICK_PLACE_OBSERVATIONS;
    static constexpr std::size_t ACTIONS = PICK_PLACE_ACTIONS;

    PickPlaceEnvironment(std::filesystem::path const& p_scenario,
                         double p_spread,
                         std::uint32_t p_max_steps);
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
    robotik::StepResult step(std::span<float const> p_action,
                             std::span<float> p_observation) override;

private:

    compages::world::World m_world;
    std::unique_ptr<PickPlaceMission> m_mission;
    std::unique_ptr<robotik::Simulation> m_simulation;
    robotik::JointGroup* m_arm = nullptr;
    robotik::VacuumGripper* m_gripper = nullptr;
    std::uint32_t m_max_steps;
    std::uint32_t m_steps = 0;
    bool m_grasped = false;
};
