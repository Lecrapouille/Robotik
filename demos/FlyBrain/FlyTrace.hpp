// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

// @file FlyTrace.hpp
// @brief One recorded episode: the seed, then every observation and action.
#pragma once

#include "FlyTypes.hpp"

#include <filesystem>
#include <vector>

// ****************************************************************************
//! @brief A fly episode stored as JSON, so it can be replayed without a brain.
// ****************************************************************************
struct FlyTrace
{
    //!< Master seed of the episode.
    std::uint64_t seed = 0;
    //!< Step the actions were computed with (SI: s).
    double dt = 0.01;
    //!< Observation read before each action. Same length as @ref actions.
    std::vector<FlyObservation> observations;
    //!< Action applied at each step, in order.
    std::vector<FlyAction> actions;

    // ------------------------------------------------------------------------
    //! @brief Writes a JSON object with the seed, the step and the two lists.
    // ------------------------------------------------------------------------
    void save(std::filesystem::path const& p_path) const;

    // ------------------------------------------------------------------------
    //! @brief Reads a file written by @ref save.
    // ------------------------------------------------------------------------
    [[nodiscard]] static FlyTrace load(std::filesystem::path const& p_path);
};
