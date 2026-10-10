// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "RlMission.hpp"

#include "PickPlaceControl.hpp"

#include "Robotik/Robot/Actuators.hpp"
#include "Robotik/Runtime/Simulation.hpp"

#include <algorithm>

#define RL_PAUSE_AFTER_S 1.2

void RlMission::reset(robotik::Simulation& p_simulation, robotik::Seed p_seed)
{
    m_arm = p_simulation.robot().actuators().find<robotik::JointGroup>("arm");
    m_gripper = p_simulation.robot().actuators().first<robotik::VacuumGripper>();
    ready = m_arm != nullptr && m_gripper != nullptr;
    steps = 0;
    episode_return = 0.0f;
    m_grasped = false;
    done = false;
    success = false;
    m_pause = 0.0;
    m_noise = robotik::Random(p_seed.derive("policy"));
    if (!ready)
    {
        return;
    }
    readyPosture(p_simulation.robot(), *m_gripper);
    p_simulation.observe();
    writePickPlaceObservation(p_simulation,
                              *m_gripper,
                              0,
                              static_cast<std::uint32_t>(max_steps),
                              observation);
}

robotik::Status RlMission::status(robotik::Simulation const& /*p_simulation*/) const
{
    if (!ready)
    {
        return robotik::Status::FAILURE;
    }
    if (done && !repeat)
    {
        return success ? robotik::Status::SUCCESS : robotik::Status::FAILURE;
    }
    return robotik::Status::RUNNING;
}

void RlMission::act(robotik::Simulation& p_simulation, double p_dt)
{
    if (!ready)
    {
        p_simulation.step(Seconds(0.01));
        return;
    }
    if (done)
    {
        if (!repeat)
        {
            return;
        }
        m_pause += p_dt;
        if (m_pause >= RL_PAUSE_AFTER_S)
        {
            p_simulation.reset(robotik::Seed{ p_simulation.seed().value + 1u });
        }
        return;
    }

    if (converged)
    {
        pickPlacePolicy(observation, action, 1.0f, nullptr);
    }
    else
    {
        pickPlacePolicy(observation, action, mix, &m_noise);
        mix = std::min(1.0f, mix + PICK_PLACE_TRAIN_STEP);
        if (mix >= 1.0f)
        {
            converged = true;
        }
    }
    applyPickPlaceAction(p_simulation, *m_arm, *m_gripper, action);
    p_simulation.observe();
    ++steps;
    writePickPlaceObservation(p_simulation,
                              *m_gripper,
                              steps,
                              static_cast<std::uint32_t>(max_steps),
                              observation);

    float reward = 0.0f;
    robotik::WorldModel const& beliefs = p_simulation.worldModel();
    if (auto const* cube = beliefs.find(PICK_PLACE_CUBE))
    {
        robotik::Vector3 const tip = m_gripper->tip(p_simulation.robot());
        if (m_gripper->holding())
        {
            auto const* box = beliefs.find(PICK_PLACE_BOX);
            robotik::Vector3 const above =
                box != nullptr
                    ? robotik::Vector3{ box->position.x, box->position.y, box->position.z + 0.1 }
                    : tip;
            reward = 1.0f - static_cast<float>(robotik::norm(cube->position - above));
            if (!m_grasped)
            {
                m_grasped = true;
                reward += 2.0f;
            }
        }
        else
        {
            robotik::Vector3 const top{ cube->position.x, cube->position.y, cube->position.z + 0.02 };
            reward = -static_cast<float>(robotik::norm(tip - top));
        }
    }
    if (cubeInBox(p_simulation, *m_gripper))
    {
        reward += 10.0f;
        done = true;
        success = true;
    }
    episode_return += reward;
    if (!done && steps >= static_cast<std::uint32_t>(max_steps))
    {
        done = true;
    }
    if (done)
    {
        if (success)
        {
            ++delivered;
        }
        else
        {
            ++failed;
        }
        m_pause = 0.0;
    }
}
