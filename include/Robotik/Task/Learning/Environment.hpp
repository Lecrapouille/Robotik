// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file Environment.hpp
/// Reinforcement learning environments and a pool running N of them in
/// parallel.
///
/// Observations and actions are flat float vectors written in place: an
/// environment step allocates nothing, and the pool stores all of them in
/// contiguous row-major buffers ready to hand to a learner. Their layout is
/// described by a @ref Space: a name, bounds and a unit per dimension.
///
/// An environment is one simulated world; an agent is something that acts in
/// it. Several environments (@ref EnvironmentPool) collect experience faster
/// or compare conditions; several agents in one environment interact. The
/// two combine, and are not to be confused.
#pragma once

#include "Robotik/Task/Learning/Space.hpp"
#include "Robotik/Common/Random.hpp"

#include <condition_variable>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <span>
#include <thread>
#include <vector>

namespace robotik
{

// ****************************************************************************
/// Why an episode ended. Completion, time limit and technical stop are
/// different events, for learning (bootstrap or not) and for evaluation.
// ****************************************************************************
enum class EpisodeEnd : std::uint8_t
{
    None,      ///< Still running.
    Success,   ///< The task is done: a terminal state.
    Failure,   ///< The task cannot be done any more: a terminal state.
    TimeLimit, ///< Cut by the step budget: not a terminal state.
    Aborted,   ///< Technical stop (invalid state, solver error): discard.
};

// ----------------------------------------------------------------------------
/// @param p_end Value to name.
/// @return Its name, for logs.
// ----------------------------------------------------------------------------
[[nodiscard]] char const* toString(EpisodeEnd p_end);

// ****************************************************************************
/// Outcome of one environment step.
///
/// @code
/// robotik::StepResult const r = env.step(action, observation);
/// total += r.reward;
/// if (r.done())
///     env.reset(next_seed, observation);
/// @endcode
// ****************************************************************************
struct StepResult
{
    /// Reward of this step.
    float reward = 0.0f;
    /// Why the episode ended, or @c None.
    EpisodeEnd end = EpisodeEnd::None;
};

/// @return True when the task ended (success or unrecoverable failure).
[[nodiscard]] inline bool terminated(StepResult const& p_result)
{
    return p_result.end == EpisodeEnd::Success || p_result.end == EpisodeEnd::Failure;
}

/// @return True when the episode was cut; the task did not end.
[[nodiscard]] inline bool truncated(StepResult const& p_result)
{
    return p_result.end == EpisodeEnd::TimeLimit || p_result.end == EpisodeEnd::Aborted;
}

/// @return True when the episode ended, for any reason.
[[nodiscard]] inline bool done(StepResult const& p_result)
{
    return p_result.end != EpisodeEnd::None;
}

// ****************************************************************************
/// One simulated world an agent acts in.
///
/// Same seed, same actions: same observations and rewards.
///
/// @code
/// class Bandit final : public robotik::Environment
/// {
/// public:
///     std::size_t observationSize() const override { return 1; }
///     std::size_t actionSize() const override { return 1; }
///     void reset(robotik::Seed, std::span<float> p_obs) override { p_obs[0] =
///     0.0f; } robotik::StepResult step(std::span<float const> p_action,
///                              std::span<float> p_obs) override
///     {
///         p_obs[0] = p_action[0];
///         return { p_action[0] > 0.5f ? 1.0f : 0.0f,
///         robotik::EpisodeEnd::Success };
///     }
/// };
/// @endcode
// ****************************************************************************
class Environment
{
public:

    virtual ~Environment() = default;

    /// @return Number of floats of an observation.
    [[nodiscard]] virtual std::size_t observationSize() const = 0;
    /// @return Number of floats of an action.
    [[nodiscard]] virtual std::size_t actionSize() const = 0;

    /// @return Layout of the observation; unnamed and unbounded by default.
    [[nodiscard]] virtual Space observationSpace() const
    {
        return Space::unnamed("o", observationSize());
    }

    /// @return Layout of the action; unnamed and unbounded by default.
    [[nodiscard]] virtual Space actionSpace() const
    {
        return Space::unnamed("a", actionSize());
    }

    // -------------------------------------------------------------------------
    /// Starts an episode and writes its first observation.
    /// @param p_seed Seed of the episode: same seed, same episode.
    /// @param p_observation Buffer of @ref observationSize floats to fill.
    // -------------------------------------------------------------------------
    virtual void reset(Seed p_seed, std::span<float> p_observation) = 0;

    // -------------------------------------------------------------------------
    /// Applies an action and writes the next observation.
    /// @param p_action @ref actionSize floats.
    /// @param p_observation Buffer of @ref observationSize floats to fill.
    /// @return The reward and whether the episode ended.
    // -------------------------------------------------------------------------
    virtual StepResult step(std::span<float const> p_action,
                            std::span<float> p_observation) = 0;
};

// ****************************************************************************
/// Summary of a finished episode.
///
/// @code
/// for (robotik::Episode const& e : pool.episodes())
///     std::cout << "env " << e.environment << " #" << e.index << ": "
///               << e.reward << " in " << e.steps << " steps ("
///               << robotik::toString(e.end) << ")\n";
/// @endcode
// ****************************************************************************
struct Episode
{
    /// Index of the environment in the pool.
    std::uint32_t environment = 0;
    /// Episode number in this environment, from 0.
    std::uint32_t index = 0;
    /// Seed of the episode, to replay it alone.
    Seed seed;
    /// Sum of the rewards (the return).
    double reward = 0.0;
    /// Number of steps.
    std::uint32_t steps = 0;
    /// Why it ended.
    EpisodeEnd end = EpisodeEnd::None;
};

/// @return True when the task ended (success or failure), not cut.
[[nodiscard]] inline bool terminated(Episode const& p_episode)
{
    return p_episode.end == EpisodeEnd::Success || p_episode.end == EpisodeEnd::Failure;
}

// ****************************************************************************
/// N environments stepped together on worker threads.
///
/// Episode @c k of environment @c i is seeded with
/// @c master.derive(i).derive(k): any episode can be replayed alone. A
/// finished environment restarts at once; its observation row then holds the
/// first observation of the next episode and @ref episodes logs the old one.
///
/// @code
/// robotik::EnvironmentPool pool(16, [](std::size_t) {
///     return std::make_unique<MyEnvironment>(); }, robotik::Seed{ 1 });
/// pool.reset();
/// for (int t = 0; t < 1000; ++t) {
///     policy(pool.observations(), pool.actions()); // fill every row
///     pool.step();
///     learn(pool.rewards(), pool.dones());
/// }
/// @endcode
// ****************************************************************************
class EnvironmentPool
{
public:

    /// Builds environment @c i of the pool.
    using Factory = std::function<std::unique_ptr<Environment>(std::size_t)>;

    /// @param p_count Number of environments.
    /// @param p_factory Builds each environment from its index.
    /// @param p_master Seed every episode seed is derived from.
    /// @param p_threads Worker threads; 0 picks the hardware concurrency.
    EnvironmentPool(std::size_t p_count,
                    Factory const& p_factory,
                    Seed p_master,
                    std::size_t p_threads = 0);
    /// Stops and joins the worker threads.
    ~EnvironmentPool();

    EnvironmentPool(EnvironmentPool const&) = delete;
    EnvironmentPool& operator=(EnvironmentPool const&) = delete;

    /// @return Number of environments.
    [[nodiscard]] std::size_t size() const
    {
        return m_environments.size();
    }

    /// @return Floats per observation row.
    [[nodiscard]] std::size_t observationSize() const
    {
        return m_observation_size;
    }

    /// @return Floats per action row.
    [[nodiscard]] std::size_t actionSize() const
    {
        return m_action_size;
    }

    /// Restarts every environment at its episode 0.
    void reset();

    /// Steps every environment with its row of @ref actions.
    void step();

    // --- Buffers (row i belongs to environment i) ----------------------------

    /// @return Every action row, to fill before @ref step.
    [[nodiscard]] std::span<float> actions()
    {
        return m_actions;
    }

    /// @return The action row of environment @p_index.
    [[nodiscard]] std::span<float> action(std::size_t p_index)
    {
        return { m_actions.data() + p_index * m_action_size, m_action_size };
    }

    /// @return Every observation row.
    [[nodiscard]] std::span<float const> observations() const
    {
        return m_observations;
    }

    /// @return The observation row of environment @p_index.
    [[nodiscard]] std::span<float const> observation(std::size_t p_index) const
    {
        return { m_observations.data() + p_index * m_observation_size,
                 m_observation_size };
    }

    /// @return The reward of the last step of each environment.
    [[nodiscard]] std::span<float const> rewards() const
    {
        return m_rewards;
    }

    /// @return 1 where the last step ended an episode.
    [[nodiscard]] std::span<std::uint8_t const> dones() const
    {
        return m_dones;
    }

    /// @return Seed of the running episode of environment @p_index.
    [[nodiscard]] Seed seed(std::size_t p_index) const
    {
        return m_master.derive(p_index).derive(m_indices[p_index]);
    }

    /// @return Finished episodes, in completion order.
    [[nodiscard]] std::span<Episode const> episodes() const
    {
        return m_episodes;
    }

    /// @return Environment @p_index, e.g. to render it.
    [[nodiscard]] Environment& environment(std::size_t p_index)
    {
        return *m_environments[p_index];
    }

private:

    /// Loop of a worker thread: waits for a generation, then @ref run.
    void work(std::size_t p_worker);
    /// Resets or steps the environments of worker @p_worker.
    void run(std::size_t p_worker);
    /// Steps environment @p_index; restarts it when the episode ended.
    void advance(std::size_t p_index);
    /// Starts the next episode of environment @p_index.
    void restart(std::size_t p_index);
    /// Runs one generation on every worker and waits for them.
    void dispatch(bool p_reset);

private:

    /// Seed every episode seed is derived from.
    Seed m_master;
    /// Floats per observation row.
    std::size_t m_observation_size = 0;
    /// Floats per action row.
    std::size_t m_action_size = 0;
    /// The environments.
    std::vector<std::unique_ptr<Environment>> m_environments;

    /// Action rows.
    std::vector<float> m_actions;
    /// Observation rows.
    std::vector<float> m_observations;
    /// Last reward of each environment.
    std::vector<float> m_rewards;
    /// 1 where the last step ended an episode.
    std::vector<std::uint8_t> m_dones;

    // Running episode of each environment.
    /// Episode number.
    std::vector<std::uint32_t> m_indices;
    /// Sum of the rewards so far.
    std::vector<double> m_returns;
    /// Steps so far.
    std::vector<std::uint32_t> m_steps;
    /// Episode each environment ended at the last step (read if done).
    std::vector<Episode> m_finished_episodes;
    /// Every finished episode.
    std::vector<Episode> m_episodes;

    // Workers: environment i runs on worker i % workers.
    /// Worker threads; empty runs on the calling thread.
    std::vector<std::thread> m_workers;
    /// Guards the fields below.
    std::mutex m_mutex;
    /// Wakes the workers for a new generation.
    std::condition_variable m_start;
    /// Wakes the caller when every worker is done.
    std::condition_variable m_finished;
    /// Number of the current generation.
    std::uint64_t m_generation = 0;
    /// Workers still running the generation.
    std::size_t m_pending = 0;
    /// The generation resets instead of stepping.
    bool m_reset = false;
    /// The workers must exit.
    bool m_stop = false;
};

} // namespace robotik
