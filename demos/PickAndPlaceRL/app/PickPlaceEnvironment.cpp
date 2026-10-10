// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "PickPlaceEnvironment.hpp"

#include "Robotik/Robot/Actuators.hpp"
#include "Robotik/Scenario/Scenario.hpp"

#include <stdexcept>

#define REWARD_GRASP 2.0f
#define REWARD_SUCCESS 10.0f

PickPlaceEnvironment::PickPlaceEnvironment(
    std::filesystem::path const& p_scenario,
    double p_spread,
    std::uint32_t p_max_steps)
    : m_max_steps(p_max_steps)
{
    robotik::Scenario scenario = robotik::Scenario::load(p_scenario);
    scenario.behavior_tree.clear();
    scenario.faults.clear();
    scenario.random_faults.clear();
    for (auto& object : scenario.objects)
    {
        if (object.shape.name == PICK_PLACE_CUBE)
        {
            object.randomize[0][0] = -p_spread;
            object.randomize[0][1] = p_spread;
            object.randomize[1][0] = -p_spread;
            object.randomize[1][1] = p_spread;
        }
    }
    m_mission = std::make_unique<PickPlaceMission>(false);
    m_simulation = std::make_unique<robotik::Simulation>(
        m_world, std::move(scenario), nullptr, m_mission.get());
    m_arm = m_simulation->robot().actuators().find<robotik::JointGroup>("arm");
    m_gripper = m_simulation->robot().actuators().first<robotik::VacuumGripper>();
    if (m_arm == nullptr || m_gripper == nullptr)
    {
        throw std::runtime_error(
            "The scenario needs an 'arm' joint group and a vacuum gripper");
    }
    readyPosture(m_simulation->robot(), *m_gripper);
}

PickPlaceEnvironment::~PickPlaceEnvironment() = default;

void PickPlaceEnvironment::reset(robotik::Seed p_seed,
                                 std::span<float> p_observation)
{
    m_simulation->reset(p_seed);
    m_steps = 0;
    m_grasped = false;
    writePickPlaceObservation(
        *m_simulation, *m_gripper, m_steps, m_max_steps, p_observation);
}

robotik::StepResult
PickPlaceEnvironment::step(std::span<float const> p_action,
                           std::span<float> p_observation)
{
    applyPickPlaceAction(*m_simulation, *m_arm, *m_gripper, p_action);
    ++m_steps;
    writePickPlaceObservation(
        *m_simulation, *m_gripper, m_steps, m_max_steps, p_observation);

    robotik::WorldModel const& beliefs = m_simulation->worldModel();
    robotik::Vector3 const cube = beliefs.find(PICK_PLACE_CUBE)->position;
    robotik::Vector3 const box = beliefs.find(PICK_PLACE_BOX)->position;
    robotik::Vector3 const tip = m_gripper->tip(m_simulation->robot());

    robotik::StepResult result;
    if (m_gripper->holding())
    {
        robotik::Vector3 const above{ box.x, box.y, box.z + 0.1 };
        result.reward = 1.0f - static_cast<float>(robotik::norm(cube - above));
        if (!m_grasped)
        {
            m_grasped = true;
            result.reward += REWARD_GRASP;
        }
    }
    else
    {
        robotik::Vector3 const top{ cube.x, cube.y, cube.z + 0.02 };
        result.reward = -static_cast<float>(robotik::norm(tip - top));
    }
    if (cubeInBox(*m_simulation, *m_gripper))
    {
        result.reward += REWARD_SUCCESS;
        result.terminated = true;
    }
    result.truncated = !result.terminated && m_steps >= m_max_steps;
    return result;
}
