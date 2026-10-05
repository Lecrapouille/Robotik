// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "Robotik/Environment/Environment.hpp"

#include <algorithm>
#include <stdexcept>

namespace robotik
{

EnvironmentPool::EnvironmentPool(std::size_t p_count,
                                 Factory const& p_factory,
                                 Seed p_master,
                                 std::size_t p_threads)
    : m_master(p_master)
{
    if (p_count == 0u)
    {
        throw std::invalid_argument("An environment pool needs environments");
    }
    for (std::size_t i = 0; i < p_count; ++i)
    {
        m_environments.push_back(p_factory(i));
    }
    m_observation_size = m_environments.front()->observationSize();
    m_action_size = m_environments.front()->actionSize();
    for (auto const& environment : m_environments)
    {
        if (environment->observationSize() != m_observation_size ||
            environment->actionSize() != m_action_size)
        {
            throw std::invalid_argument("Pooled environments differ in size");
        }
    }

    m_actions.assign(p_count * m_action_size, 0.0f);
    m_observations.assign(p_count * m_observation_size, 0.0f);
    m_rewards.assign(p_count, 0.0f);
    m_dones.assign(p_count, 0u);
    m_indices.assign(p_count, 0u);
    m_returns.assign(p_count, 0.0);
    m_steps.assign(p_count, 0u);
    m_finished_episodes.assign(p_count, Episode{});

    std::size_t workers =
        p_threads != 0u ? p_threads : std::max(1u, std::thread::hardware_concurrency());
    workers = std::min(workers, p_count);
    if (workers > 1u)
    {
        for (std::size_t w = 0; w < workers; ++w)
        {
            m_workers.emplace_back([this, w]() { work(w); });
        }
    }
}

EnvironmentPool::~EnvironmentPool()
{
    {
        std::lock_guard const lock(m_mutex);
        m_stop = true;
    }
    m_start.notify_all();
    for (std::thread& worker : m_workers)
    {
        worker.join();
    }
}

void EnvironmentPool::restart(std::size_t p_index)
{
    std::span<float> const row(m_observations.data() + p_index * m_observation_size,
                               m_observation_size);
    m_environments[p_index]->reset(seed(p_index), row);
    m_returns[p_index] = 0.0;
    m_steps[p_index] = 0u;
}

void EnvironmentPool::advance(std::size_t p_index)
{
    std::span<float> const row(m_observations.data() + p_index * m_observation_size,
                               m_observation_size);
    StepResult const result = m_environments[p_index]->step(action(p_index), row);
    m_rewards[p_index] = result.reward;
    m_returns[p_index] += static_cast<double>(result.reward);
    ++m_steps[p_index];
    m_dones[p_index] = result.done() ? 1u : 0u;
    if (result.done())
    {
        m_finished_episodes[p_index] = { static_cast<std::uint32_t>(p_index),
                                         m_indices[p_index],
                                         seed(p_index),
                                         m_returns[p_index],
                                         m_steps[p_index],
                                         result.terminated };
        ++m_indices[p_index];
        restart(p_index);
    }
}

void EnvironmentPool::run(std::size_t p_worker)
{
    std::size_t const stride = std::max<std::size_t>(1u, m_workers.size());
    for (std::size_t i = p_worker; i < m_environments.size(); i += stride)
    {
        if (m_reset)
        {
            restart(i);
        }
        else
        {
            advance(i);
        }
    }
}

void EnvironmentPool::work(std::size_t p_worker)
{
    std::uint64_t seen = 0;
    while (true)
    {
        {
            std::unique_lock lock(m_mutex);
            m_start.wait(lock, [&]() { return m_stop || m_generation != seen; });
            if (m_stop)
            {
                return;
            }
            seen = m_generation;
        }
        run(p_worker);
        {
            std::lock_guard const lock(m_mutex);
            --m_pending;
        }
        m_finished.notify_one();
    }
}

void EnvironmentPool::dispatch(bool p_reset)
{
    m_reset = p_reset;
    if (m_workers.empty())
    {
        run(0u);
        return;
    }
    {
        std::lock_guard const lock(m_mutex);
        m_pending = m_workers.size();
        ++m_generation;
    }
    m_start.notify_all();
    std::unique_lock lock(m_mutex);
    m_finished.wait(lock, [this]() { return m_pending == 0u; });
}

void EnvironmentPool::reset()
{
    std::fill(m_indices.begin(), m_indices.end(), 0u);
    std::fill(m_dones.begin(), m_dones.end(), 0u);
    std::fill(m_rewards.begin(), m_rewards.end(), 0.0f);
    m_episodes.clear();
    dispatch(true);
}

void EnvironmentPool::step()
{
    dispatch(false);

    // Logged in environment order: the log does not depend on the threads.
    for (std::size_t i = 0; i < m_environments.size(); ++i)
    {
        if (m_dones[i] != 0u)
        {
            m_episodes.push_back(m_finished_episodes[i]);
        }
    }
}

} // namespace robotik
