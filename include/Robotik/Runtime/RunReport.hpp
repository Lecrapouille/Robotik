// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

//! @file RunReport.hpp
//! @brief What a finished run leaves behind: outcome, verdicts and metrics,
//! with what is needed to replay it (scenario, case, seed).
#pragma once

#include "Robotik/Scenario/Assertions.hpp"

#include <cstdint>
#include <iosfwd>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace robotik
{

class Simulation;

// ****************************************************************************
//! @brief One run of one case with one seed.
// ****************************************************************************
struct RunRecord
{
    std::string case_id;
    std::string scenario;
    std::uint64_t seed = 0;
    std::string outcome;
    //!< Simulated time at the end (s).
    double time = 0.0;
    bool passed = false;
    std::vector<Check> checks;
    //!< Name and value; nothing when it could not be measured.
    std::vector<std::pair<std::string, std::optional<double>>> metrics;
    //!< Why the run could not be built or started, if it was not.
    std::string error;

    // -------------------------------------------------------------------------
    //! @brief Snapshot of a concluded simulation; @p_metrics are measured now.
    // -------------------------------------------------------------------------
    [[nodiscard]] static RunRecord capture(Simulation const& p_simulation,
                                           std::string p_case,
                                           std::vector<std::string> const& p_metrics);
};

// ****************************************************************************
//! @brief Every run of an experiment (or the single run of a scenario).
// ****************************************************************************
struct RunReport
{
    std::string experiment;
    std::vector<RunRecord> runs;

    [[nodiscard]] bool passed() const;

    //! @brief One line per run, then the failed assertions.
    void writeText(std::ostream& p_out) const;

    //! @brief The whole report as JSON (for scripts comparing runs).
    void writeJson(std::ostream& p_out) const;
};

} // namespace robotik
