// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

//! @file Environment.hpp
//! @brief Reinforcement learning environments and a pool running N of them in
//! parallel.
//!
//! Observations and actions are flat float vectors written in place: an
//! environment step allocates nothing, and the pool stores all of them in
//! contiguous row-major buffers ready to hand to a learner.
#pragma once

#include "Robotik/Math/Random.hpp"

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
//! @brief Outcome of one environment step.
// ****************************************************************************
struct StepResult
{
    float reward = 0.0f;
    //!< The task ended (success or unrecoverable failure).
    bool terminated = false;
    //!< The episode was cut (time limit), the task did not end.
    bool truncated = false;

    [[nodiscard]] bool done() const
    {
        return terminated || truncated;
    }
};

// ****************************************************************************
//! @brief One simulated world an agent acts in.
//!
//! Same seed, same actions: same observations and rewards.
// ****************************************************************************
class Environment
{
public:

    virtual ~Environment() = default;

    [[nodiscard]] virtual std::size_t observationSize() const = 0;
    [[nodiscard]] virtual std::size_t actionSize() const = 0;

    //! @brief Starts an episode and writes its first observation.
    virtual void reset(Seed p_seed, std::span<float> p_observation) = 0;

    //! @brief Applies @p_action and writes the next observation.
    virtual StepResult step(std::span<float const> p_action,
                            std::span<float> p_observation) = 0;
};

// ****************************************************************************
//! @brief Summary of a finished episode.
// ****************************************************************************
struct Episode
{
    std::uint32_t environment = 0;
    //!< Episode number in this environment, from 0.
    std::uint32_t index = 0;
    Seed seed;
    double reward = 0.0;
    std::uint32_t steps = 0;
    bool terminated = false;
};

// ****************************************************************************
//! @brief N environments stepped together on worker threads.
//!
//! Episode @c k of environment @c i is seeded with
//! @c master.derive(i).derive(k): any episode can be replayed alone. A
//! finished environment restarts at once; its observation row then holds the
//! first observation of the next episode and @ref episodes logs the old one.
//!
//! @code
//! robotik::EnvironmentPool pool(16, [](std::size_t) {
//!     return std::make_unique<MyEnvironment>(); }, robotik::Seed{ 1 });
//! pool.reset();
//! for (int t = 0; t < 1000; ++t) {
//!     policy(pool.observations(), pool.actions()); // fill every row
//!     pool.step();
//!     learn(pool.rewards(), pool.dones());
//! }
//! @endcode
// ****************************************************************************
class EnvironmentPool
{
public:

    using Factory = std::function<std::unique_ptr<Environment>(std::size_t)>;

    //! @param p_threads Worker threads; 0 picks the hardware concurrency.
    EnvironmentPool(std::size_t p_count,
                    Factory const& p_factory,
                    Seed p_master,
                    std::size_t p_threads = 0);
    ~EnvironmentPool();

    EnvironmentPool(EnvironmentPool const&) = delete;
    EnvironmentPool& operator=(EnvironmentPool const&) = delete;

    [[nodiscard]] std::size_t size() const
    {
        return m_environments.size();
    }

    [[nodiscard]] std::size_t observationSize() const
    {
        return m_observation_size;
    }

    [[nodiscard]] std::size_t actionSize() const
    {
        return m_action_size;
    }

    //! @brief Restarts every environment at its episode 0.
    void reset();

    //! @brief Steps every environment with its row of @ref actions.
    void step();

    // --- Buffers (row i belongs to environment i) ----------------------------

    [[nodiscard]] std::span<float> actions()
    {
        return m_actions;
    }

    [[nodiscard]] std::span<float> action(std::size_t p_index)
    {
        return { m_actions.data() + p_index * m_action_size, m_action_size };
    }

    [[nodiscard]] std::span<float const> observations() const
    {
        return m_observations;
    }

    [[nodiscard]] std::span<float const> observation(std::size_t p_index) const
    {
        return { m_observations.data() + p_index * m_observation_size,
                 m_observation_size };
    }

    [[nodiscard]] std::span<float const> rewards() const
    {
        return m_rewards;
    }

    //! @brief 1 where the last step ended an episode.
    [[nodiscard]] std::span<std::uint8_t const> dones() const
    {
        return m_dones;
    }

    //! @brief Seed of the running episode of environment @p_index.
    [[nodiscard]] Seed seed(std::size_t p_index) const
    {
        return m_master.derive(p_index).derive(m_indices[p_index]);
    }

    //! @brief Finished episodes, in completion order.
    [[nodiscard]] std::span<Episode const> episodes() const
    {
        return m_episodes;
    }

    //! @brief Direct access, e.g. to render one environment.
    [[nodiscard]] Environment& environment(std::size_t p_index)
    {
        return *m_environments[p_index];
    }

private:

    void work(std::size_t p_worker);
    void run(std::size_t p_worker);
    void advance(std::size_t p_index);
    void restart(std::size_t p_index);
    void dispatch(bool p_reset);

private:

    Seed m_master;
    std::size_t m_observation_size = 0;
    std::size_t m_action_size = 0;
    std::vector<std::unique_ptr<Environment>> m_environments;

    std::vector<float> m_actions;
    std::vector<float> m_observations;
    std::vector<float> m_rewards;
    std::vector<std::uint8_t> m_dones;

    // Running episode of each environment.
    std::vector<std::uint32_t> m_indices;
    std::vector<double> m_returns;
    std::vector<std::uint32_t> m_steps;
    std::vector<Episode> m_finished_episodes;
    std::vector<Episode> m_episodes;

    // Workers: environment i runs on worker i % workers.
    std::vector<std::thread> m_workers;
    std::mutex m_mutex;
    std::condition_variable m_start;
    std::condition_variable m_finished;
    std::uint64_t m_generation = 0;
    std::size_t m_pending = 0;
    bool m_reset = false;
    bool m_stop = false;
};

} // namespace robotik
