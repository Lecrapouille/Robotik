// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "Robotik/Task/Learning/SimulationEnvironment.hpp"

#include "Robotik/Task/Cell.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace robotik
{

/// The agent drives and the reward judges: the behavior tree of the cell does
/// not run.
static CellDescription forLearning(CellDescription p_description)
{
    p_description.controller.behavior_tree.clear();
    return p_description;
}

SimulationEnvironment::SimulationEnvironment(CellDescription p_description,
                                             Parts p_parts,
                                             EnvironmentTiming p_timing)
    : m_parts(std::move(p_parts)), m_timing(p_timing)
{
    if (m_parts.observation == nullptr || m_parts.action == nullptr ||
        m_parts.reward == nullptr)
    {
        throw std::invalid_argument(
            "An environment needs an observation, an action and a reward");
    }
    double const ratio =
        m_timing.control_period.value() / m_timing.physics_step.value();
    if (!(m_timing.physics_step.value() > 0.0) || ratio < 1.0 - 1e-9 ||
        std::abs(ratio - std::round(ratio)) > 1e-9)
    {
        throw std::invalid_argument(
            "The control period must be a whole number of physics steps");
    }
    m_substeps = static_cast<std::uint32_t>(std::lround(ratio));
    m_clock.max_steps = m_timing.max_steps;
    m_observation_space = m_parts.observation->space();
    m_action_space = m_parts.action->space();

    m_simulation =
        std::make_unique<Cell>(m_world,
                               forLearning(std::move(p_description)),
                                     nullptr,
                                     m_parts.mission.get());
    m_parts.observation->setup(*m_simulation);
    m_parts.action->setup(*m_simulation);
    m_parts.reward->setup(*m_simulation);
}

SimulationEnvironment::~SimulationEnvironment() = default;

void SimulationEnvironment::reset(Seed p_seed, std::span<float> p_observation)
{
    m_abort.clear();
    m_clock.steps = 0;
    m_simulation->reset(p_seed);
    m_parts.action->reset(*m_simulation);
    m_parts.reward->reset(*m_simulation);
    m_parts.observation->reset(*m_simulation);
    m_parts.observation->observe(*m_simulation, m_clock, p_observation);
}

StepResult SimulationEnvironment::step(std::span<float const> p_action,
                                       std::span<float> p_observation)
{
    StepResult result;
    try
    {
        m_parts.action->apply(p_action, *m_simulation);
        for (std::uint32_t i = 0; i < m_substeps; ++i)
        {
            m_simulation->step(m_timing.physics_step);
        }
        ++m_clock.steps;
        result = m_parts.reward->evaluate(*m_simulation);
        if (!result.done() && m_clock.steps >= m_timing.max_steps)
        {
            result.end = EpisodeEnd::TimeLimit;
        }
        m_parts.observation->observe(*m_simulation, m_clock, p_observation);
    }
    catch (std::exception const& error)
    {
        // A solver or a part gave up: the episode says nothing about the
        // policy. The pool restarts it like any other ended episode.
        m_abort = error.what();
        result.reward = 0.0f;
        result.end = EpisodeEnd::Aborted;
        std::fill(p_observation.begin(), p_observation.end(), 0.0f);
    }
    return result;
}

} // namespace robotik
