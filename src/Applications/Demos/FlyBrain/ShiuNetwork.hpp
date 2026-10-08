// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

// @file ShiuNetwork.hpp
// @brief Leaky integrate-and-fire network with the constants of Shiu et al.
// 2024.
//
// The equations are those of model.py in
// github.com/philshiu/Drosophila_brain_model:
//
//   dv/dt = (v_rest - v + g) / t_membrane
//   dg/dt = -g / tau
//   a spike resets v to v_reset and g to 0
//   a synapse adds (count * w_syn) to g after t_delay
//   a Poisson input adds (w_syn * f_poi) to v
//
// Weights are the column "Excitatory x Connectivity": positive excitatory,
// negative inhibitory. Neuron order is the row order of Completeness_*.csv.
#pragma once

#include "Robotik/Math/Random.hpp"

#include <cstdint>
#include <filesystem>
#include <vector>

// ****************************************************************************
//! @brief One Shiu network: synapses, Poisson drives and the spike counters.
// ****************************************************************************
class ShiuNetwork
{
public:

    //!< Integration step (SI: s). Eighteen of them make the synaptic delay.
    static constexpr double SUBSTEP_S = 1.0e-4;
    //!< Resting potential (SI: mV).
    static constexpr double V_REST_MV = -52.0;
    //!< Potential just after a spike (SI: mV).
    static constexpr double V_RESET_MV = -52.0;
    //!< Spike threshold (SI: mV).
    static constexpr double V_THRESHOLD_MV = -45.0;
    //!< Membrane time constant (SI: ms).
    static constexpr double T_MEMBRANE_MS = 20.0;
    //!< Synaptic conductance time constant (SI: ms).
    static constexpr double TAU_SYNAPSE_MS = 5.0;
    //!< Time a neuron ignores its input after a spike (SI: ms).
    static constexpr double T_REFRACTORY_MS = 2.2;
    //!< Synaptic delay (SI: ms).
    static constexpr double T_DELAY_MS = 1.8;
    //!< Synaptic weight of one connection count (SI: mV).
    static constexpr double W_SYN_MV = 0.275;
    //!< Scale of a Poisson input, as in model.py.
    static constexpr double F_POISSON = 250.0;

    // ------------------------------------------------------------------------
    //! @brief Allocates @p_neurons cells at rest. Synapses added afterwards
    //! wait for @ref build.
    // ------------------------------------------------------------------------
    void resize(std::uint32_t p_neurons);

    // ------------------------------------------------------------------------
    //! @brief Queues one synapse. @p_count is excitatory times connectivity
    //! and may be negative. Call @ref build before @ref step.
    // ------------------------------------------------------------------------
    void addSynapse(std::uint32_t p_pre, std::uint32_t p_post, float p_count);

    // ------------------------------------------------------------------------
    //! @brief Marks @p_neuron as a Poisson input. @ref setRate then drives it.
    // ------------------------------------------------------------------------
    void addDrive(std::uint32_t p_neuron);

    // ------------------------------------------------------------------------
    //! @brief Holds @p_neuron at rest. It neither spikes nor relays.
    // ------------------------------------------------------------------------
    void silence(std::uint32_t p_neuron);

    // ------------------------------------------------------------------------
    //! @brief Freezes the queued synapses into a sparse matrix.
    // ------------------------------------------------------------------------
    void build();

    // ------------------------------------------------------------------------
    //! @brief Resting potentials, empty spike counters, Poisson seed taken
    //! from @p_seed. Builds first when @ref build was not called.
    // ------------------------------------------------------------------------
    void reset(robotik::Seed p_seed);

    // ------------------------------------------------------------------------
    //! @brief Poisson rate of a drive neuron (SI: Hz). Other neurons ignore it.
    // ------------------------------------------------------------------------
    void setRate(std::uint32_t p_neuron, double p_hertz);

    // ------------------------------------------------------------------------
    //! @brief Integrates @p_dt seconds. Spike counters keep growing until
    //! @ref clearSpikes.
    // ------------------------------------------------------------------------
    void step(double p_dt);

    // ------------------------------------------------------------------------
    //! @brief Cells allocated by @ref resize or @ref loadEdges.
    // ------------------------------------------------------------------------
    [[nodiscard]] std::uint32_t neurons() const
    {
        return static_cast<std::uint32_t>(m_voltage.size());
    }

    // ------------------------------------------------------------------------
    //! @brief Synapses stored after @ref build.
    // ------------------------------------------------------------------------
    [[nodiscard]] std::uint64_t synapses() const
    {
        return m_column.size();
    }

    // ------------------------------------------------------------------------
    //! @brief Spikes of @p_neuron since the last @ref clearSpikes.
    // ------------------------------------------------------------------------
    [[nodiscard]] std::uint32_t spikes(std::uint32_t p_neuron) const
    {
        return m_spikes[p_neuron];
    }

    // ------------------------------------------------------------------------
    //! @brief Zeroes every spike counter. Does not change the potentials.
    // ------------------------------------------------------------------------
    void clearSpikes();

    // ------------------------------------------------------------------------
    //! @brief Loads an edge list. A non-numeric first line is skipped.
    //! Columns: presynaptic index, postsynaptic index, weight.
    // ------------------------------------------------------------------------
    [[nodiscard]] static ShiuNetwork
    loadEdges(std::filesystem::path const& p_path);

private:

    // ------------------------------------------------------------------------
    //! @brief One substep of @ref SUBSTEP_S, including delayed synapses.
    // ------------------------------------------------------------------------
    void substep();

    // ------------------------------------------------------------------------
    //! @brief Puts @p_neuron in the active set. A resting cell is absent
    //! from it, so a quiet connectome does not walk every neuron.
    // ------------------------------------------------------------------------
    void wake(std::uint32_t p_neuron);

    //!< Presynaptic indices queued before @ref build.
    std::vector<std::uint32_t> m_pending_pre;
    //!< Postsynaptic indices queued before @ref build.
    std::vector<std::uint32_t> m_pending_post;
    //!< Weights queued before @ref build.
    std::vector<float> m_pending_count;

    //!< Cells. Zero until @ref resize.
    std::uint32_t m_count = 0;
    //!< CSR row pointer, size neurons + 1.
    std::vector<std::uint32_t> m_row;
    //!< CSR column index of each synapse.
    std::vector<std::uint32_t> m_column;
    //!< CSR connection count of each synapse. @ref W_SYN_MV scales it on a
    //!< spike.
    std::vector<float> m_weight;
    //!< Membrane potential (SI: mV).
    std::vector<float> m_voltage;
    //!< Synaptic conductance (SI: mV).
    std::vector<float> m_conductance;
    //!< Refractory time still to wait (SI: ms).
    std::vector<float> m_refractory;
    //!< Requested Poisson rate (SI: Hz).
    std::vector<float> m_rate;
    //!< 1 when the cell is a Poisson input.
    std::vector<std::uint8_t> m_drive;
    //!< 1 when the cell is held silent.
    std::vector<std::uint8_t> m_silent;
    //!< Indices of the drive neurons, for the Poisson loop.
    std::vector<std::uint32_t> m_drives;
    //!< Delayed conductance kicks, one slot per substep of the delay, per
    //!< neuron. Only the indices in @ref m_touch are non-zero.
    std::vector<float> m_ring;
    //!< 1 when that delay slot holds a kick still to deliver.
    std::vector<std::uint8_t> m_dirty;
    //!< Neurons with a kick in each delay slot. Sparse stand-in for a scan
    //!< of @ref m_ring.
    std::vector<std::vector<std::uint32_t>> m_touch;
    //!< Substep index at which @ref m_touch last listed that neuron.
    std::vector<std::uint32_t> m_listed;
    //!< Cells that are not at rest. Integrated instead of the whole population.
    std::vector<std::uint32_t> m_awake;
    //!< 1 when the cell is already in @ref m_awake.
    std::vector<std::uint8_t> m_awake_mark;
    //!< Spikes since @ref clearSpikes.
    std::vector<std::uint32_t> m_spikes;
    //!< Neurons that spiked on the last substep.
    std::vector<std::uint32_t> m_fired;
    //!< Poisson draws.
    robotik::Random m_random{ robotik::Seed{} };
    //!< Seconds not yet consumed by a whole substep.
    double m_residual = 0.0;
    //!< Substeps taken, used as the ring cursor.
    std::uint32_t m_step = 0;
    //!< True after @ref build.
    bool m_built = false;
};
