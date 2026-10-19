// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "RlMission.hpp"

#include "PickPlaceControl.hpp"

#include "Robotik/Robot/Actuators.hpp"
#include "Robotik/Package/Simulation.hpp"

#include <algorithm>
#include <cmath>

#define RL_PAUSE_AFTER_S 1.2

void RlMission::reset(robotik::Simulation& p_simulation, robotik::Seed p_seed)
{
    steps = 0;
    episode_return = 0.0f;
    done = false;
    success = false;
    m_pause = 0.0;
    m_noise = robotik::Random(p_seed.derive("policy"));
    try
    {
        // Every episode starts again from the ready posture.
        m_action.setup(p_simulation);
        m_observation.setup(p_simulation);
        m_reward.setup(p_simulation);
        ready = true;
    }
    catch (std::exception const&)
    {
        ready = false;
        return;
    }
    m_reward.reset(p_simulation);
    p_simulation.observe();
    m_observation.observe(p_simulation,
                          { 0u, static_cast<std::uint32_t>(max_steps) },
                          observation);
}

robotik::Status
RlMission::status(robotik::Simulation const& /*p_simulation*/) const
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
        p_simulation.step(robotik::Seconds(PICK_PLACE_PHYSICS_STEP_S));
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
    m_action.apply(action, p_simulation);
    auto const substeps = static_cast<int>(
        std::lround(PICK_PLACE_CONTROL_PERIOD_S / PICK_PLACE_PHYSICS_STEP_S));
    for (int i = 0; i < substeps; ++i)
    {
        p_simulation.step(robotik::Seconds(PICK_PLACE_PHYSICS_STEP_S));
    }
    p_simulation.observe();
    ++steps;
    robotik::StepResult const result = m_reward.evaluate(p_simulation);
    m_observation.observe(p_simulation,
                          { steps, static_cast<std::uint32_t>(max_steps) },
                          observation);
    episode_return += result.reward;
    if (result.end == robotik::EpisodeEnd::Success)
    {
        done = true;
        success = true;
    }
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
