// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file SimulationEnvironment.hpp
/// @brief A reinforcement learning environment over a scenario, assembled
/// from three parts that can be swapped independently.
///
/// @li @ref ObservationBuilder: what the agent may know, as a @ref Space;
/// @li @ref ActionAdapter: how a float vector becomes robot commands;
/// @li @ref RewardEvaluator: how a step is scored, and when the task ends.
///
/// The environment owns the simulation: it resets the world from a seed,
/// holds each action for a fixed control period, truncates at the step
/// budget and aborts the episode on a technical error. No window, no input
/// event: the same environment runs in a pool, a test or a Python bridge.
///
/// @code
/// robotik::SimulationEnvironment::Parts parts;
/// parts.observation = std::make_unique<MyObservation>();
/// parts.action = std::make_unique<MyAction>();
/// parts.reward = std::make_unique<MyReward>();
/// robotik::SimulationEnvironment env(
///     robotik::Scenario::load(path, assets), std::move(parts),
///     robotik::EnvironmentTiming{ .control_period = Seconds(0.1), .max_steps = 100 });
/// @endcode
#pragma once

#include "Robotik/Environment/Environment.hpp"
#include "Robotik/Runtime/Mission.hpp"
#include "Robotik/Scenario/Scenario.hpp"

#include "Compages/World/World.hpp"

#include <cstdint>
#include <memory>
#include <span>
#include <string>

namespace robotik
{

class Simulation;

// ****************************************************************************
/// @brief Progress of the running episode, as the parts see it.
// ****************************************************************************
struct EpisodeClock
{
    std::uint32_t steps = 0;
    std::uint32_t max_steps = 0;
};

// ****************************************************************************
/// @brief Writes what the agent may know. Prefer sensors and beliefs; mark
/// the dimensions read from the ground truth as privileged in @ref space.
// ****************************************************************************
class ObservationBuilder
{
public:

    virtual ~ObservationBuilder() = default;

    [[nodiscard]] virtual Space space() const = 0;

    /// @brief Once, after the simulation is built.
    virtual void setup(Simulation& /*p_simulation*/) {}

    /// @brief At each episode start, after the world is reset.
    virtual void reset(Simulation& /*p_simulation*/) {}

    virtual void observe(Simulation const& p_simulation,
                         EpisodeClock const& p_clock,
                         std::span<float> p_observation) = 0;
};

// ****************************************************************************
/// @brief Turns an action vector into robot commands. It does not advance
/// time: the environment holds the commands for the control period.
// ****************************************************************************
class ActionAdapter
{
public:

    virtual ~ActionAdapter() = default;

    [[nodiscard]] virtual Space space() const = 0;

    /// @brief Once, after the simulation is built (e.g. a ready posture).
    virtual void setup(Simulation& /*p_simulation*/) {}

    /// @brief At each episode start, after the world is reset.
    virtual void reset(Simulation& /*p_simulation*/) {}

    virtual void apply(std::span<float const> p_action, Simulation& p_simulation) = 0;
};

// ****************************************************************************
/// @brief Scores a step from the simulation alone (never from the display).
/// @ref evaluate returns the reward and, when the task is over,
/// @ref EpisodeEnd::Success or @ref EpisodeEnd::Failure.
// ****************************************************************************
class RewardEvaluator
{
public:

    virtual ~RewardEvaluator() = default;

    virtual void setup(Simulation const& /*p_simulation*/) {}
    virtual void reset(Simulation const& /*p_simulation*/) {}

    [[nodiscard]] virtual StepResult evaluate(Simulation const& p_simulation) = 0;
};

// ****************************************************************************
/// @brief How time flows in an episode.
// ****************************************************************************
struct EnvironmentTiming
{
    /// Step of the simulation.
    Seconds physics_step{ 0.01 };
    /// Duration an action is held: a whole number of physics steps.
    Seconds control_period{ 0.1 };
    /// Steps before the episode is truncated (@ref EpisodeEnd::TimeLimit).
    std::uint32_t max_steps = 100;
};

// ****************************************************************************
/// @brief @ref Environment running a scenario @ref Simulation.
///
/// The scenario keeps its world, faults and perception policy. Its
/// controller, timeout, stop condition and assertions are dropped: the agent
/// drives, and the reward judges the episode.
// ****************************************************************************
class SimulationEnvironment final: public Environment
{
public:

    struct Parts
    {
        std::unique_ptr<ObservationBuilder> observation;
        std::unique_ptr<ActionAdapter> action;
        std::unique_ptr<RewardEvaluator> reward;
        /// Optional: perception, metrics, dressing of the scenario.
        std::unique_ptr<Mission> mission;
    };

    /// @throws std::invalid_argument when a part is missing or the control
    /// period is not a positive multiple of the physics step.
    SimulationEnvironment(Scenario p_scenario, Parts p_parts, EnvironmentTiming p_timing);
    ~SimulationEnvironment() override;

    [[nodiscard]] std::size_t observationSize() const override
    {
        return m_observation_space.size();
    }

    [[nodiscard]] std::size_t actionSize() const override
    {
        return m_action_space.size();
    }

    [[nodiscard]] Space observationSpace() const override
    {
        return m_observation_space;
    }

    [[nodiscard]] Space actionSpace() const override
    {
        return m_action_space;
    }

    [[nodiscard]] EnvironmentTiming const& timing() const
    {
        return m_timing;
    }

    void reset(Seed p_seed, std::span<float> p_observation) override;
    StepResult step(std::span<float const> p_action, std::span<float> p_observation) override;

    /// @brief The simulation, e.g. to render one environment of a pool.
    [[nodiscard]] Simulation& simulation()
    {
        return *m_simulation;
    }

    /// @brief Why the last episode was aborted, empty otherwise.
    [[nodiscard]] std::string const& abortReason() const
    {
        return m_abort;
    }

private:

    compages::world::World m_world;
    Parts m_parts;
    EnvironmentTiming m_timing;
    std::uint32_t m_substeps = 1;
    Space m_observation_space;
    Space m_action_space;
    EpisodeClock m_clock;
    std::string m_abort;
    /// Declared last: destroyed before the mission and the world it uses.
    std::unique_ptr<Simulation> m_simulation;
};

} // namespace robotik
