// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file EpisodeLog.hpp
/// What a learning or evaluation run must keep to be compared and replayed:
/// the episodes with their seeds, and what produced them.
#pragma once

#include "Robotik/Environment/Environment.hpp"

#include <cstdint>
#include <iosfwd>
#include <span>
#include <string>

namespace robotik
{

// ****************************************************************************
/// Context of a list of episodes. Any episode replays from the scenario, the
/// master seed (or its own seed), the policy and the version.
///
/// Fill the fields once after the run, then write the episodes to a JSON
/// file. Two logs can then be compared to check that a change did not alter
/// the results.
///
/// @code
/// robotik::EpisodeLog log;
/// log.scenario = env.simulation().scenario().id;
/// log.policy = "random";
/// log.master_seed = 42;
/// log.observation = env.observationSpace();
/// log.action = env.actionSpace();
/// log.control_period = env.timing().control_period.value();
/// log.max_steps = env.timing().max_steps;
/// std::ofstream file("episodes.json");
/// log.write(file, episodes);
/// @endcode
// ****************************************************************************
struct EpisodeLog
{
    /// Id of the scenario the episodes ran.
    std::string scenario;
    /// Free name of the policy that chose the actions (e.g. "random").
    std::string policy;
    /// Seed from which every episode seed is derived.
    std::uint64_t master_seed = 0;
    /// Layout of the observation vector.
    Space observation;
    /// Layout of the action vector.
    Space action;
    /// Time between two actions [s].
    double control_period = 0.0;
    /// Episode length limit, in steps.
    std::uint32_t max_steps = 0;

    // ------------------------------------------------------------------------
    /// Writes the log as JSON: the context, RobotIK version and commit, then
    /// one entry per episode (environment, index, seed, return, steps, end).
    /// @param p_out Stream receiving the JSON.
    /// @param p_episodes Episodes to write, in the order given.
    // ------------------------------------------------------------------------
    void write(std::ostream& p_out, std::span<Episode const> p_episodes) const;
};

} // namespace robotik
