// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file SimulationEnvironment.hpp
/// A reinforcement learning environment over a robot cell, assembled from
/// three parts that can be swapped independently.
///
/// @li @ref robotik::ObservationBuilder — what the agent may know, as a @ref
///     robotik::Space;
/// @li @ref robotik::ActionAdapter — how a float vector becomes robot commands;
/// @li @ref robotik::RewardEvaluator — how a step is scored, and when the task
///     ends.
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
///     robotik::loadScenario(path, assets).take(), // a CellDescription
///     std::move(parts),
///     robotik::EnvironmentTiming{ .control_period = Seconds(0.1),
///                                 .max_steps = 100 });
/// @endcode
///
/// @par Contract
/// - Promises: an episode starts from the seed given to @c reset; a technical
///   error aborts the episode instead of the program.
/// - Expects: the three parts are given.
/// - Errors: a missing part, or a control period that is not a positive
///   multiple of the physics step, throws @c std::invalid_argument.
#pragma once

#include "Robotik/Task/Learning/Environment.hpp"
#include "Robotik/Task/Mission.hpp"
#include "Robotik/Task/CellDescription.hpp"

#include "Compages/World/World.hpp"

#include <cstdint>
#include <memory>
#include <span>
#include <string>

namespace robotik
{

class Cell;

// ****************************************************************************
/// Progress of the running episode, as the parts see it.
///
/// @code
/// float const time_left = 1.0f - float(p_clock.steps) /
/// float(p_clock.max_steps);
/// @endcode
// ****************************************************************************
struct EpisodeClock
{
    /// Steps done in the episode.
    std::uint32_t steps = 0;
    /// Step budget of the episode.
    std::uint32_t max_steps = 0;
};

// ****************************************************************************
/// Writes what the agent may know. Prefer sensors and beliefs; mark the
/// dimensions read from the ground truth as privileged in @ref space.
///
/// @code
/// class JointsObservation final : public robotik::ObservationBuilder
/// {
/// public:
///     robotik::Space space() const override
///     {
///         return robotik::Space().add("q0", -3.2f, 3.2f, "rad").add("q1",
///         -3.2f, 3.2f, "rad");
///     }
///     void observe(robotik::Cell const& p_cell, robotik::EpisodeClock
///     const&,
///                  std::span<float> p_obs) override
///     {
///         auto const q = p_cell.robot().joints().positions();
///         p_obs[0] = float(q[0]);
///         p_obs[1] = float(q[1]);
///     }
/// };
/// @endcode
// ****************************************************************************
class ObservationBuilder
{
public:

    virtual ~ObservationBuilder() = default;

    /// @return Layout of the observation.
    [[nodiscard]] virtual Space space() const = 0;

    /// Called once, after the simulation is built.
    virtual void setup(Cell& /*p_cell*/)
    {
        // Nothing to do by default.
    }

    /// Called at each episode start, after the world is reset.
    virtual void reset(Cell& /*p_cell*/)
    {
        // Nothing to do by default.
    }

    // -------------------------------------------------------------------------
    /// Writes the observation of the current state.
    /// @param p_cell The running simulation.
    /// @param p_clock Progress of the episode.
    /// @param p_observation Buffer of @c space().size() floats to fill.
    // -------------------------------------------------------------------------
    virtual void observe(Cell const& p_cell,
                         EpisodeClock const& p_clock,
                         std::span<float> p_observation) = 0;
};

// ****************************************************************************
/// Turns an action vector into robot commands. It does not advance time:
/// the environment holds the commands for the control period.
///
/// @code
/// class JointTargets final : public robotik::ActionAdapter
/// {
/// public:
///     robotik::Space space() const override { return
///     robotik::Space::unnamed("q", 2); } void apply(std::span<float const>
///     p_action, robotik::Cell& p_cell) override
///     {
///         auto& joints = p_cell.robot().joints();
///         joints.moveTo(0, p_action[0]);
///         joints.moveTo(1, p_action[1]);
///     }
/// };
/// @endcode
// ****************************************************************************
class ActionAdapter
{
public:

    virtual ~ActionAdapter() = default;

    /// @return Layout of the action.
    [[nodiscard]] virtual Space space() const = 0;

    /// Called once, after the simulation is built (e.g. a ready posture).
    virtual void setup(Cell& /*p_cell*/)
    {
        // Nothing to do by default.
    }

    /// Called at each episode start, after the world is reset.
    virtual void reset(Cell& /*p_cell*/)
    {
        // Nothing to do by default.
    }

    // -------------------------------------------------------------------------
    /// Writes the robot commands of an action.
    /// @param p_action @c space().size() floats.
    /// @param p_cell The running simulation.
    // -------------------------------------------------------------------------
    virtual void apply(std::span<float const> p_action,
                       Cell& p_cell) = 0;
};

// ****************************************************************************
/// Scores a step from the simulation alone (never from the display).
/// @ref evaluate returns the reward and, when the task is over,
/// @ref EpisodeEnd::Success or @ref EpisodeEnd::Failure.
///
/// The reward may read the ground truth (@c Cell::pose): it is not
/// part of what the agent sees.
///
/// @code
/// robotik::StepResult Reach::evaluate(robotik::Cell const& p_cell)
/// {
///     double const d = distanceToTarget(p_cell);
///     if (d < 0.01)
///         return { 1.0f, robotik::EpisodeEnd::Success };
///     return { float(-d), robotik::EpisodeEnd::None };
/// }
/// @endcode
// ****************************************************************************
class RewardEvaluator
{
public:

    virtual ~RewardEvaluator() = default;

    /// Called once, after the simulation is built.
    virtual void setup(Cell const& /*p_cell*/)
    {
        // Nothing to do by default.
    }
    /// Called at each episode start, after the world is reset.
    virtual void reset(Cell const& /*p_cell*/)
    {
        // Nothing to do by default.
    }

    // -------------------------------------------------------------------------
    /// Scores the step that just ended.
    /// @param p_cell The running simulation.
    /// @return The reward, and the end of the task if it is over.
    // -------------------------------------------------------------------------
    [[nodiscard]] virtual StepResult
    evaluate(Cell const& p_cell) = 0;
};

// ****************************************************************************
/// How time flows in an episode.
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
/// @ref Environment running a @ref Cell (example in the file
/// comment).
///
/// The cell keeps its world, faults and perception policy. Its behavior tree
/// is dropped: the agent drives, and the reward judges the episode.
// ****************************************************************************
class SimulationEnvironment final: public Environment
{
public:

    // -------------------------------------------------------------------------
    /// The swappable parts of the environment.
    // -------------------------------------------------------------------------
    struct Parts
    {
        /// What the agent sees (required).
        std::unique_ptr<ObservationBuilder> observation;
        /// How the agent acts (required).
        std::unique_ptr<ActionAdapter> action;
        /// How a step is scored (required).
        std::unique_ptr<RewardEvaluator> reward;
        /// Optional: perception, metrics, dressing of the cell.
        std::unique_ptr<Mission> mission;
    };

    // -------------------------------------------------------------------------
    /// Builds the cell and calls the @c setup of the
    /// parts.
    /// @param p_description Cell to build.
    /// @param p_parts Observation, action, reward and optional mission.
    /// @param p_timing Physics step, control period and step budget.
    /// @throws std::invalid_argument when a part is missing or the control
    ///   period is not a positive multiple of the physics step.
    // -------------------------------------------------------------------------
    SimulationEnvironment(CellDescription p_description,
                          Parts p_parts,
                          EnvironmentTiming p_timing);
    ~SimulationEnvironment() override;

    /// @return Floats of an observation.
    [[nodiscard]] std::size_t observationSize() const override
    {
        return m_observation_space.size();
    }

    /// @return Floats of an action.
    [[nodiscard]] std::size_t actionSize() const override
    {
        return m_action_space.size();
    }

    /// @return Layout of the observation, from the observation part.
    [[nodiscard]] Space observationSpace() const override
    {
        return m_observation_space;
    }

    /// @return Layout of the action, from the action part.
    [[nodiscard]] Space actionSpace() const override
    {
        return m_action_space;
    }

    /// @return How time flows in an episode.
    [[nodiscard]] EnvironmentTiming const& timing() const
    {
        return m_timing;
    }

    /// Resets the world with @p_seed, then the parts; writes the first
    /// observation.
    void reset(Seed p_seed, std::span<float> p_observation) override;

    /// Applies the action, runs the physics for one control period, scores
    /// the step; truncates at the budget, aborts on a technical error.
    StepResult step(std::span<float const> p_action,
                    std::span<float> p_observation) override;

    /// @return The simulation, e.g. to render one environment of a pool.
    [[nodiscard]] Cell& simulation()
    {
        return *m_simulation;
    }

    /// @return Why the last episode was aborted, empty otherwise.
    [[nodiscard]] std::string const& abortReason() const
    {
        return m_abort;
    }

private:

    /// ECS world of the simulation.
    compages::world::World m_world;
    /// Observation, action, reward and mission.
    Parts m_parts;
    /// Physics step, control period and step budget.
    EnvironmentTiming m_timing;
    /// Physics steps per control period.
    std::uint32_t m_substeps = 1;
    /// Layout of the observation.
    Space m_observation_space;
    /// Layout of the action.
    Space m_action_space;
    /// Progress of the running episode.
    EpisodeClock m_clock;
    /// Why the last episode was aborted.
    std::string m_abort;
    /// Declared last: destroyed before the mission and the world it uses.
    std::unique_ptr<Cell> m_simulation;
};

} // namespace robotik
