// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file EpisodeLog.hpp
/// @brief What a learning or evaluation run must keep to be compared and
/// replayed: the episodes with their seeds, and what produced them.
#pragma once

#include "Robotik/Environment/Environment.hpp"

#include <cstdint>
#include <iosfwd>
#include <span>
#include <string>

namespace robotik
{

// ****************************************************************************
/// @brief Context of a list of episodes. Any episode replays from the
/// scenario, the master seed (or its own seed), the policy and the version.
// ****************************************************************************
struct EpisodeLog
{
    std::string scenario;
    std::string policy;
    std::uint64_t master_seed = 0;
    Space observation;
    Space action;
    double control_period = 0.0;
    std::uint32_t max_steps = 0;

    /// @brief JSON: the context, RobotIK version and commit, then one entry
    /// per episode (environment, index, seed, return, steps, end).
    void write(std::ostream& p_out, std::span<Episode const> p_episodes) const;
};

} // namespace robotik
