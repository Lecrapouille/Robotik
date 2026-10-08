// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "FlyBrain.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_map>

namespace
{

// Sensor roles first, then motor roles. The embedded circuit uses the
// index as the neuron index; a connectome overwrites m_role.
enum Role : int
{
    InLeft = 0,
    InCenter,
    InRight,
    InSeekLeft,
    InSeekRight,
    InTooLow,
    InTooHigh,
    InBias,
    OutForward,
    OutTurnLeft,
    OutTurnRight,
    OutUp,
    OutDown,
    RoleCount
};

constexpr double EMA_TAU_S = 0.05;

//! @brief Clamps to [0, 1]. Forward speed has no reverse.
double clamp01(double p_value)
{
    return std::clamp(p_value, 0.0, 1.0);
}

//! @brief Clamps to [-1, 1]. Turn and lift are signed.
double clamp11(double p_value)
{
    return std::clamp(p_value, -1.0, 1.0);
}

//! @brief Binding-file name to a @ref Role. Unknown names are refused.
int roleOf(std::string const& p_name)
{
    static std::unordered_map<std::string, int> const roles = {
        { "visual_left", InLeft },      { "visual_center", InCenter },
        { "visual_right", InRight },    { "seek_left", InSeekLeft },
        { "seek_right", InSeekRight },  { "too_low", InTooLow },
        { "too_high", InTooHigh },      { "bias", InBias },
        { "forward", OutForward },      { "turn_left", OutTurnLeft },
        { "turn_right", OutTurnRight }, { "lift_up", OutUp },
        { "lift_down", OutDown },
    };
    auto const found = roles.find(p_name);
    if (found == roles.end())
    {
        throw std::runtime_error("Unknown fly binding role '" + p_name + "'");
    }
    return found->second;
}

} // namespace

FlyBrain::FlyBrain(Kind p_kind, float p_cruise_altitude)
    : m_kind(p_kind), m_cruise(p_cruise_altitude)
{
    // Every role starts present, so the embedded circuit needs no file.
    // A reflex brain never builds a network.
    for (int role = 0; role < RoleCount; ++role)
    {
        m_role[role].push_back(static_cast<std::uint32_t>(role));
        m_have_role[role] = true;
    }

    // Build the embedded circuit if it's a Shiu brain.
    if (m_kind == Kind::Shiu)
    {
        buildEmbedded();
    }
}

void FlyBrain::buildEmbedded()
{
    // Thirteen neurons. Counts are "Excitatory x Connectivity": one
    // presynaptic spike must be enough to fire the postsynaptic cell.
    m_network = std::make_unique<ShiuNetwork>();
    auto synapse = [&](int p_pre, int p_post, float p_count)
    {
        m_network->addSynapse(static_cast<std::uint32_t>(p_pre),
                              static_cast<std::uint32_t>(p_post),
                              p_count);
    };

    // Cruise, unless the centre eye sees an obstacle.
    synapse(InBias, OutForward, 400.0f);
    synapse(InCenter, OutForward, -650.0f);

    // Each eye turns the fly away from itself, and inhibits the turn toward it.
    synapse(InLeft, OutTurnRight, 700.0f);
    synapse(InRight, OutTurnLeft, 700.0f);
    synapse(InLeft, OutTurnLeft, -300.0f);
    synapse(InRight, OutTurnRight, -300.0f);

    // Food beacon. A head-on obstacle (centre eye, no side winner) prefers
    // left.
    synapse(InSeekLeft, OutTurnLeft, 420.0f);
    synapse(InSeekRight, OutTurnRight, 420.0f);
    synapse(InCenter, OutTurnLeft, 220.0f);

    // Altitude error about the food height.
    synapse(InTooLow, OutUp, 400.0f);
    synapse(InTooHigh, OutDown, 400.0f);

    // The sensor roles are Poisson inputs. Motor cells spike on
    // their own synapses.
    for (int role = InLeft; role <= InBias; ++role)
    {
        m_network->addDrive(static_cast<std::uint32_t>(role));
    }
    m_network->resize(static_cast<std::uint32_t>(RoleCount));
    m_connectome = false;
}

std::uint32_t FlyBrain::neurons() const
{
    return m_network ? m_network->neurons() : 0u;
}

std::uint64_t FlyBrain::synapses() const
{
    return m_network ? m_network->synapses() : 0u;
}

void FlyBrain::loadConnectome(std::filesystem::path const& p_edges,
                              std::filesystem::path const& p_binding)
{
    // Replace the embedded circuit with a connectome.
    m_kind = Kind::Shiu;
    m_network = std::make_unique<ShiuNetwork>(ShiuNetwork::loadEdges(p_edges));
    bind(p_binding);
    m_connectome = true;
}

void FlyBrain::bind(std::filesystem::path const& p_path)
{
    // Read the binding file.
    std::ifstream file(p_path);
    if (!file)
    {
        throw std::runtime_error("Cannot open binding '" + p_path.string() +
                                 "'");
    }

    // Drop the embedded indices. A repeated line adds a neuron to that role.
    for (std::vector<std::uint32_t>& neurons : m_role)
    {
        neurons.clear();
    }
    for (bool& have : m_have_role)
    {
        have = false;
    }

    // For each role, read the neuron index.
    std::string role;
    unsigned long index = 0;
    while (file >> role)
    {
        // Skip comments.
        if (!role.empty() && role[0] == '#')
        {
            std::string rest;
            std::getline(file, rest);
            continue;
        }

        // Silence a neuron.
        if (role == "silence")
        {
            if (!(file >> index))
            {
                throw std::runtime_error("silence in '" + p_path.string() +
                                         "' has no neuron index");
            }
            m_network->silence(static_cast<std::uint32_t>(index));
            continue;
        }

        // Read the neuron index.
        if (!(file >> index))
        {
            throw std::runtime_error("Binding role '" + role +
                                     "' has no neuron index");
        }

        // Bind the role to the neuron index. Another line for the same
        // role adds a cell: motor spikes are summed, inputs share the rate.
        int const which = roleOf(role);
        m_role[which].push_back(static_cast<std::uint32_t>(index));
        m_have_role[which] = true;

        // Only the sensor roles are Poisson inputs. Motor cells spike on
        // their own synapses.
        if (which <= InBias)
        {
            m_network->addDrive(static_cast<std::uint32_t>(index));
        }
    }
}

void FlyBrain::reset(robotik::Seed p_seed)
{
    // Clear the exponential averages.
    std::fill(std::begin(m_ema), std::end(m_ema), 0.0f);
    if (m_network)
    {
        m_network->reset(p_seed.derive("shiu"));
    }
}

void FlyBrain::stimulate(FlyObservation const& p_observation)
{
    // Set the rate of the role if it's present.
    auto rate = [&](int p_role, double p_hertz)
    {
        if (m_have_role[p_role])
        {
            double const hertz = std::clamp(p_hertz, 0.0, 400.0);
            for (std::uint32_t neuron : m_role[p_role])
            {
                m_network->setRate(neuron, hertz);
            }
        }
    };

    rate(InLeft, static_cast<double>(p_observation.visual_left) * 360.0);
    rate(InCenter, static_cast<double>(p_observation.visual_center) * 360.0);
    rate(InRight, static_cast<double>(p_observation.visual_right) * 360.0);
    rate(InSeekLeft,
         std::max(static_cast<double>(p_observation.target_bearing), 0.0) /
             1.2 * 300.0);
    rate(InSeekRight,
         std::max(-static_cast<double>(p_observation.target_bearing), 0.0) /
             1.2 * 300.0);
    rate(InTooLow,
         std::max(static_cast<double>(m_cruise - p_observation.altitude), 0.0) *
             220.0);
    rate(InTooHigh,
         std::max(static_cast<double>(p_observation.altitude - m_cruise), 0.0) *
             220.0);
    rate(InBias, 180.0);
}

FlyAction FlyBrain::readout(double p_dt)
{
    // Accumulate the spikes over the time step.
    auto accumulate = [&](int p_role, int p_ema)
    {
        if (!m_have_role[p_role] || p_dt <= 0.0)
        {
            return;
        }

        // Convert the spikes to hertz. Several cells on one role add up,
        // so a food synapse and an eye synapse can share a turn.
        double spikes = 0.0;
        for (std::uint32_t neuron : m_role[p_role])
        {
            spikes += static_cast<double>(m_network->spikes(neuron));
        }
        float const hertz = static_cast<float>(spikes / p_dt);

        // Exponential average.
        float const alpha =
            static_cast<float>(1.0 - std::exp(-p_dt / EMA_TAU_S));
        m_ema[p_ema] += (hertz - m_ema[p_ema]) * alpha;
    };

    // Accumulate the spikes for each motor role.
    accumulate(OutForward, 0);
    accumulate(OutTurnLeft, 1);
    accumulate(OutTurnRight, 2);
    accumulate(OutUp, 3);
    accumulate(OutDown, 4);
    m_network->clearSpikes();

    // Convert the hertz to an action.
    FlyAction action;
    action.forward =
        static_cast<float>(clamp01(static_cast<double>(m_ema[0]) / 140.0));
    action.turn = static_cast<float>(clamp11(
        (static_cast<double>(m_ema[1]) - static_cast<double>(m_ema[2])) /
        100.0));
    action.lift = static_cast<float>(clamp11(
        (static_cast<double>(m_ema[3]) - static_cast<double>(m_ema[4])) /
        80.0));

    return action;
}

FlyAction FlyBrain::reflex(FlyObservation const& p_observation) const
{
    // Calculate the threat level.
    float const threat = std::max(
        p_observation.visual_center,
        std::max(p_observation.visual_left, p_observation.visual_right));

    // Calculate the avoidance level.
    float avoid = p_observation.visual_right - p_observation.visual_left;

    // Head-on: the two eyes agree, so pick a side instead of flying in.
    if (p_observation.visual_center > 0.22f && std::fabs(avoid) < 0.06f)
    {
        avoid = 1.0f;
    }

    // Calculate the seek level.
    float const seek =
        std::clamp(p_observation.target_bearing / 0.8f, -1.0f, 1.0f);

    // Calculate the action.
    FlyAction action;
    action.turn = static_cast<float>(
        clamp11(static_cast<double>(seek) * static_cast<double>(1.0f - threat) +
                1.6 * static_cast<double>(avoid)));

    // If the obstacle is close, stop chasing the food and turn hard.
    if (p_observation.visual_center > 0.4f)
    {
        action.turn =
            static_cast<float>(clamp11(static_cast<double>(avoid) * 2.0));
        action.forward = 0.4f;
    }
    else if (threat > 0.25f)
    {
        action.forward = 0.7f;
    }
    else
    {
        action.forward = 1.0f;
    }

    // Calculate the lift level.
    action.lift = static_cast<float>(
        clamp11(static_cast<double>(m_cruise - p_observation.altitude) * 2.0));

    return action;
}

FlyAction FlyBrain::update(FlyObservation const& p_observation, Seconds p_dt)
{
    // If it's a reflex brain or no network, use the reflex policy.
    if (m_kind == Kind::Reflex || m_network == nullptr)
    {
        return reflex(p_observation);
    }

    double const dt = p_dt.value();

    // Stimulate the network.
    stimulate(p_observation);

    // Step the network.
    m_network->step(dt);

    // Read out the action.
    return readout(dt);
}

robotik::Action FlyBrain::update(robotik::Observation const& p_observation,
                                 Seconds p_dt)
{
    // Convert the observation to a FlyObservation and update the brain.
    FlyAction const decided =
        update(FlyObservation::from(p_observation.values), p_dt);

    // Convert the FlyAction to a robotik::Action.
    robotik::Action action;
    action.values.resize(FlyAction::SIZE);

    // Write the decided action to the action.
    decided.write(action.values);

    return action;
}
