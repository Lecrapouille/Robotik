// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

// @file FlyBrain.hpp
// @brief Closed-loop flyer: a reflex rule, or the Shiu neuron model.
//
// Neither path knows MuJoCo, Compages or the URDF. The default Shiu circuit
// is a small sensorimotor stand-in. @ref FlyBrain::loadConnectome replaces
// it with a FlyWire edge list.
#pragma once

#include "FlyTypes.hpp"
#include "ShiuNetwork.hpp"

#include "Robotik/Agents/Agent.hpp"

#include <filesystem>
#include <memory>
#include <vector>

// ****************************************************************************
//! @brief Maps a @ref FlyObservation to a @ref FlyAction.
// ****************************************************************************
class FlyBrain final: public robotik::Agent
{
public:

    // ------------------------------------------------------------------------
    //! @brief Which decision rule @ref update runs.
    // ------------------------------------------------------------------------
    enum class Kind
    {
        Reflex, //!< Explicit avoidance rule, used to check the loop.
        Shiu,   //!< Leaky integrate-and-fire neurons of Shiu et al. 2024.
    };

    // ------------------------------------------------------------------------
    //! @brief Builds a reflex brain, or the embedded Shiu circuit.
    //!
    //! @p_cruise_altitude is the height the lift neurons try to hold (SI: m),
    //! usually the food height.
    // ------------------------------------------------------------------------
    FlyBrain(Kind p_kind, float p_cruise_altitude);

    // ------------------------------------------------------------------------
    //! @brief Clears the spike filters. Derives the Poisson seed from
    //! @p_seed when a network is loaded.
    // ------------------------------------------------------------------------
    void reset(robotik::Seed p_seed);

    // ------------------------------------------------------------------------
    //! @brief Replaces the embedded circuit with a connectome.
    //!
    //! @p_edges is the synaptic list (presynaptic index, postsynaptic index,
    //! excitatory times connectivity). @p_binding names which neuron index
    //! fills each sensor and each motor. Both files must name real neurons.
    // ------------------------------------------------------------------------
    void loadConnectome(std::filesystem::path const& p_edges,
                        std::filesystem::path const& p_binding);

    // ------------------------------------------------------------------------
    //! @brief One decision for the fly-shaped observation.
    // ------------------------------------------------------------------------
    FlyAction update(FlyObservation const& p_observation, Seconds p_dt);

    // ------------------------------------------------------------------------
    //! @brief @ref Agent entry. The observation must hold
    //! @ref FlyObservation::SIZE floats.
    // ------------------------------------------------------------------------
    robotik::Action update(robotik::Observation const& p_observation,
                           Seconds p_dt) override;

    // ------------------------------------------------------------------------
    //! @brief Rule selected at construction. A loaded connectome stays
    //! @ref Kind::Shiu.
    // ------------------------------------------------------------------------
    [[nodiscard]] Kind kind() const
    {
        return m_kind;
    }

    // ------------------------------------------------------------------------
    //! @brief True after a successful @ref loadConnectome.
    // ------------------------------------------------------------------------
    [[nodiscard]] bool connectome() const
    {
        return m_connectome;
    }

    // ------------------------------------------------------------------------
    //! @brief Cells in the loaded network. Zero for a reflex brain.
    // ------------------------------------------------------------------------
    [[nodiscard]] std::uint32_t neurons() const;

    // ------------------------------------------------------------------------
    //! @brief Synapses in the loaded network. Zero for a reflex brain.
    // ------------------------------------------------------------------------
    [[nodiscard]] std::uint64_t synapses() const;

private:

    // ------------------------------------------------------------------------
    //! @brief Wires the thirteen-neuron stand-in and marks every role present.
    // ------------------------------------------------------------------------
    void buildEmbedded();

    // ------------------------------------------------------------------------
    //! @brief Reads the role file into @ref m_role. A repeated role adds
    //! another neuron: inputs share the rate, motor spikes are summed.
    // ------------------------------------------------------------------------
    void bind(std::filesystem::path const& p_path);

    // ------------------------------------------------------------------------
    //! @brief Turns the observation into Poisson rates on the input neurons.
    // ------------------------------------------------------------------------
    void stimulate(FlyObservation const& p_observation);

    // ------------------------------------------------------------------------
    //! @brief Spike counts of the motor neurons, filtered, as a fly action.
    // ------------------------------------------------------------------------
    [[nodiscard]] FlyAction readout(double p_dt);

    // ------------------------------------------------------------------------
    //! @brief Avoidance rule used when @ref m_kind is @ref Kind::Reflex.
    // ------------------------------------------------------------------------
    [[nodiscard]] FlyAction reflex(FlyObservation const& p_observation) const;

    //!< Rule selected at construction.
    Kind m_kind;
    //!< Height the lift channels hold (SI: m).
    float m_cruise;
    //!< True when the network came from an edge list.
    bool m_connectome = false;
    //!< Empty for a reflex brain.
    std::unique_ptr<ShiuNetwork> m_network;
    //!< Neuron indices of each role. A motor role sums every index.
    std::vector<std::uint32_t> m_role[13];
    //!< True when @ref m_role holds at least one neuron.
    bool m_have_role[13]{};
    //!< Filtered motor rates: forward, turn left, turn right, up, down.
    float m_ema[5]{};
};
