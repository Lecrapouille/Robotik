// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "ShiuNetwork.hpp"

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <stdexcept>
#include <string>

namespace
{

// 1.8 ms / 0.1 ms is 18. Dividing the doubles truncates to 17, so the
// count is written out.
constexpr int DELAY_STEPS = 18;
constexpr int RING = DELAY_STEPS + 1;
constexpr double SUBSTEP_MS = ShiuNetwork::SUBSTEP_S * 1000.0;

//! @brief Poisson count. A tiny lambda is a coin flip; otherwise Knuth's
//! product of uniforms, which stops at the first draw that falls under e^-λ.
int poisson(robotik::Random& p_random, double p_lambda)
{
    if (p_lambda <= 0.0)
    {
        return 0;
    }
    if (p_lambda < 0.08)
    {
        return p_random.uniform() < p_lambda ? 1 : 0;
    }
    int count = 0;
    double const limit = std::exp(-p_lambda);
    double product = 1.0;
    do
    {
        ++count;
        product *= p_random.uniform();
    } while (product > limit);
    return count - 1;
}

} // namespace

void ShiuNetwork::resize(std::uint32_t p_neurons)
{
    m_count = p_neurons;
    m_built = false;
}

void ShiuNetwork::addSynapse(std::uint32_t p_pre,
                             std::uint32_t p_post,
                             float p_count)
{
    m_pending_pre.push_back(p_pre);
    m_pending_post.push_back(p_post);
    m_pending_count.push_back(p_count);
    m_count = std::max(m_count, std::max(p_pre, p_post) + 1u);
    m_built = false;
}

void ShiuNetwork::addDrive(std::uint32_t p_neuron)
{
    m_count = std::max(m_count, p_neuron + 1u);
    m_drives.push_back(p_neuron);
    if (p_neuron < m_drive.size())
    {
        m_drive[p_neuron] = 1;
    }
}

void ShiuNetwork::silence(std::uint32_t p_neuron)
{
    m_count = std::max(m_count, p_neuron + 1u);
    if (m_silent.size() < m_count)
    {
        m_silent.resize(m_count, 0);
    }
    m_silent[p_neuron] = 1;
}

void ShiuNetwork::build()
{
    // Histogram of outgoing edges, then the exclusive prefix that is the CSR.
    m_row.assign(static_cast<std::size_t>(m_count) + 1u, 0u);
    for (std::uint32_t pre : m_pending_pre)
    {
        ++m_row[static_cast<std::size_t>(pre) + 1u];
    }
    for (std::uint32_t neuron = 0; neuron < m_count; ++neuron)
    {
        m_row[neuron + 1u] += m_row[neuron];
    }
    m_column.assign(m_pending_pre.size(), 0u);
    m_weight.assign(m_pending_pre.size(), 0.0f);
    std::vector<std::uint32_t> cursor(m_row.begin(), m_row.end() - 1);

    // Turn the queued edges into a CSR matrix.
    for (std::size_t edge = 0; edge < m_pending_pre.size(); ++edge)
    {
        std::uint32_t const slot = cursor[m_pending_pre[edge]]++;
        m_column[slot] = m_pending_post[edge];
        m_weight[slot] = m_pending_count[edge];
    }

    // Allocate the state at rest.
    m_voltage.assign(m_count, static_cast<float>(V_REST_MV));
    m_conductance.assign(m_count, 0.0f);
    m_refractory.assign(m_count, 0.0f);
    m_rate.assign(m_count, 0.0f);
    m_drive.assign(m_count, 0);

    // Set the drives and the silent cells.
    if (m_silent.size() < m_count)
    {
        m_silent.resize(m_count, 0);
    }

    // Set the drives.
    for (std::uint32_t drive : m_drives)
    {
        if (drive < m_count)
        {
            m_drive[drive] = 1;
        }
    }

    // One slot per substep of the delay, plus the slot being delivered now.
    // The touch lists name the few cells a slot actually hits.
    m_ring.assign(static_cast<std::size_t>(RING) * m_count, 0.0f);
    m_dirty.assign(RING, 0);
    m_touch.assign(RING, {});
    m_listed.assign(m_count, 0u);
    m_awake.clear();
    m_awake_mark.assign(m_count, 0);
    m_spikes.assign(m_count, 0u);
    m_fired.clear();
    m_residual = 0.0;
    m_step = 0;
    m_built = true;
}

void ShiuNetwork::reset(robotik::Seed p_seed)
{
    if (!m_built)
    {
        build();
    }

    std::fill(
        m_voltage.begin(), m_voltage.end(), static_cast<float>(V_REST_MV));
    std::fill(m_conductance.begin(), m_conductance.end(), 0.0f);
    std::fill(m_refractory.begin(), m_refractory.end(), 0.0f);
    std::fill(m_rate.begin(), m_rate.end(), 0.0f);
    std::fill(m_ring.begin(), m_ring.end(), 0.0f);
    std::fill(m_dirty.begin(), m_dirty.end(), 0);
    std::fill(m_spikes.begin(), m_spikes.end(), 0u);
    for (std::uint32_t neuron : m_awake)
    {
        if (neuron < m_awake_mark.size())
        {
            m_awake_mark[neuron] = 0;
        }
    }
    m_awake.clear();
    for (std::vector<std::uint32_t>& touch : m_touch)
    {
        touch.clear();
    }
    std::fill(m_listed.begin(), m_listed.end(), 0u);

    m_residual = 0.0;
    m_step = 0;
    m_random = robotik::Random(p_seed);
}

void ShiuNetwork::setRate(std::uint32_t p_neuron, double p_hertz)
{
    if (p_neuron < m_rate.size())
    {
        m_rate[p_neuron] = static_cast<float>(p_hertz);
    }
}

void ShiuNetwork::clearSpikes()
{
    std::fill(m_spikes.begin(), m_spikes.end(), 0u);
}

void ShiuNetwork::step(double p_dt)
{
    if (!m_built)
    {
        build();
    }
    m_residual += p_dt;
    while (m_residual + 1.0e-12 >= SUBSTEP_S)
    {
        substep();
        m_residual -= SUBSTEP_S;
    }
}

void ShiuNetwork::wake(std::uint32_t p_neuron)
{
    if (p_neuron >= m_awake_mark.size() || m_awake_mark[p_neuron] != 0 ||
        m_silent[p_neuron] != 0)
    {
        return;
    }
    m_awake_mark[p_neuron] = 1;
    m_awake.push_back(p_neuron);
}

void ShiuNetwork::substep()
{
    // Conductance that was scheduled DELAY_STEPS ago arrives on this slot.
    // Only the neurons listed when the spike was sent are visited.
    std::uint32_t const slot = m_step % static_cast<std::uint32_t>(RING);
    if (m_dirty[slot] != 0)
    {
        float* incoming =
            m_ring.data() + static_cast<std::size_t>(slot) * m_count;
        for (std::uint32_t neuron : m_touch[slot])
        {
            float const kick = incoming[neuron];
            incoming[neuron] = 0.0f;
            if (m_silent[neuron] != 0 ||
                std::bit_cast<std::uint32_t>(kick) == 0u)
            {
                continue;
            }
            m_conductance[neuron] += kick;
            wake(neuron);
        }
        m_touch[slot].clear();
        m_dirty[slot] = 0;
    }

    // A Poisson event adds w_syn * f_poi straight to the potential.
    double const kick = W_SYN_MV * F_POISSON;
    for (std::uint32_t drive : m_drives)
    {
        if (drive >= m_count || m_silent[drive] != 0)
        {
            continue;
        }
        double const lambda = static_cast<double>(m_rate[drive]) * SUBSTEP_S;
        int const events = poisson(m_random, lambda);
        if (events > 0)
        {
            m_voltage[drive] += static_cast<float>(events * kick);
            wake(drive);
        }
    }

    // Calculate the decay and gain factors.
    double const decay_v = std::exp(-SUBSTEP_MS / T_MEMBRANE_MS);
    double const decay_g = std::exp(-SUBSTEP_MS / TAU_SYNAPSE_MS);
    double const gain = (T_MEMBRANE_MS / TAU_SYNAPSE_MS) - 1.0;
    m_fired.clear();

    // Update only the cells that left rest. A drive with no Poisson event
    // this substep is absent: at rest its update would change nothing.
    float const rest = static_cast<float>(V_REST_MV);
    std::uint32_t const rest_bits = std::bit_cast<std::uint32_t>(rest);
    std::size_t kept = 0;
    for (std::size_t index = 0; index < m_awake.size(); ++index)
    {
        std::uint32_t const neuron = m_awake[index];
        bool active = true;
        if (m_silent[neuron] != 0)
        {
            active = false;
        }
        else if (m_refractory[neuron] > 0.0f)
        {
            // The kick delivered above waits out the refractory period.
            m_refractory[neuron] -= static_cast<float>(SUBSTEP_MS);
            if (m_refractory[neuron] <= 0.0f)
            {
                m_refractory[neuron] = 0.0f;
                active = std::bit_cast<std::uint32_t>(m_conductance[neuron]) !=
                             0u ||
                         std::bit_cast<std::uint32_t>(m_voltage[neuron]) !=
                             rest_bits;
            }
        }
        else
        {
            // Calculate the new voltage and conductance.
            float const conductance = m_conductance[neuron];
            m_voltage[neuron] = static_cast<float>(
                V_REST_MV +
                (static_cast<double>(m_voltage[neuron]) - V_REST_MV) * decay_v +
                static_cast<double>(conductance) * (decay_v - decay_g) / gain);
            m_conductance[neuron] =
                static_cast<float>(static_cast<double>(conductance) * decay_g);

            // Close enough to rest that the next substep can skip the cell.
            if (std::fabs(m_conductance[neuron]) < 1.0e-5f &&
                std::fabs(m_voltage[neuron] - rest) < 1.0e-4f)
            {
                m_conductance[neuron] = 0.0f;
                m_voltage[neuron] = rest;
                active = false;
            }

            // A drive neuron has no refractory period: it is an input, not a
            // cell that just fired a spike of its own.
            if (m_voltage[neuron] >= static_cast<float>(V_THRESHOLD_MV))
            {
                m_voltage[neuron] = static_cast<float>(V_RESET_MV);
                m_conductance[neuron] = 0.0f;
                if (m_drive[neuron] == 0)
                {
                    m_refractory[neuron] = static_cast<float>(T_REFRACTORY_MS);
                    active = true;
                }
                else
                {
                    active = false;
                }
                m_fired.push_back(neuron);
                ++m_spikes[neuron];
            }
        }

        if (active)
        {
            m_awake[kept++] = neuron;
        }
        else
        {
            m_awake_mark[neuron] = 0;
        }
    }
    m_awake.resize(kept);

    // Each spike adds count * w_syn onto the postsynaptic conductance,
    // delivered DELAY_STEPS substeps from now.
    std::uint32_t const future =
        (m_step + static_cast<std::uint32_t>(DELAY_STEPS)) %
        static_cast<std::uint32_t>(RING);
    if (!m_fired.empty())
    {
        float* outgoing =
            m_ring.data() + static_cast<std::size_t>(future) * m_count;
        std::uint32_t const due = m_step + static_cast<std::uint32_t>(DELAY_STEPS);
        for (std::uint32_t neuron : m_fired)
        {
            for (std::uint32_t edge = m_row[neuron]; edge < m_row[neuron + 1u];
                 ++edge)
            {
                std::uint32_t const post = m_column[edge];
                if (m_silent[post] != 0)
                {
                    continue;
                }
                outgoing[post] += m_weight[edge] * static_cast<float>(W_SYN_MV);
                if (m_listed[post] != due)
                {
                    m_listed[post] = due;
                    m_touch[future].push_back(post);
                }
            }
        }
        m_dirty[future] = 1;
    }
    ++m_step;
}

ShiuNetwork ShiuNetwork::loadEdges(std::filesystem::path const& p_path)
{
    std::ifstream file(p_path);
    if (!file)
    {
        throw std::runtime_error("Cannot open connectome '" + p_path.string() +
                                 "'");
    }

    std::fprintf(stderr, "connectome: reading %s\n", p_path.c_str());
    ShiuNetwork network;
    if (auto const bytes = std::filesystem::file_size(p_path); bytes > 0)
    {
        std::size_t const guess = bytes / 12;
        network.m_pending_pre.reserve(guess);
        network.m_pending_post.reserve(guess);
        network.m_pending_count.reserve(guess);
    }

    // Parse the connectome file.
    std::string line;
    while (std::getline(file, line))
    {
        unsigned long pre = 0;
        unsigned long post = 0;
        double weight = 0.0;
        if (std::sscanf(line.c_str(), "%lu,%lu,%lf", &pre, &post, &weight) !=
                3 &&
            std::sscanf(line.c_str(), "%lu %lu %lf", &pre, &post, &weight) != 3)
        {
            continue;
        }

        // Add the synapse.
        network.addSynapse(static_cast<std::uint32_t>(pre),
                           static_cast<std::uint32_t>(post),
                           static_cast<float>(weight));
    }

    // Build the network. The staging lists are a second copy of every edge.
    network.build();
    network.m_pending_pre.clear();
    network.m_pending_post.clear();
    network.m_pending_count.clear();
    network.m_pending_pre.shrink_to_fit();
    network.m_pending_post.shrink_to_fit();
    network.m_pending_count.shrink_to_fit();
    std::fprintf(stderr,
                 "connectome: %u neurons, %llu synapses\n",
                 network.neurons(),
                 static_cast<unsigned long long>(network.synapses()));
    return network;
}
