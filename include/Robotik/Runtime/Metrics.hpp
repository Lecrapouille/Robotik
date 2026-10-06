// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

//! @file Metrics.hpp
//! @brief Named numbers published by a run, and the scenario assertions
//! checked against them.
//!
//! A mission publishes what it measures (@c laps, @c cross_track.max,
//! @c success_rate...), the simulation adds @c time, @c collisions and
//! @c robot.success. A scenario then asserts on them without the library
//! knowing what they mean:
//! @code
//! assert:
//!   - robot.success
//!   - time < 60
//!   - cross_track.max < 0.08
//! @endcode
#pragma once

#include <functional>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace robotik
{

// ****************************************************************************
//! @brief Flat table of named values (insertion order kept).
// ****************************************************************************
class Metrics
{
public:

    //! @brief Creates or overwrites @p_name.
    void set(std::string_view p_name, double p_value);

    //! @brief Value of @p_name, or nothing.
    [[nodiscard]] std::optional<double> get(std::string_view p_name) const;

    void clear()
    {
        m_names.clear();
        m_values.clear();
    }

    [[nodiscard]] std::span<std::string const> names() const
    {
        return m_names;
    }

    [[nodiscard]] std::span<double const> values() const
    {
        return m_values;
    }

private:

    std::vector<std::string> m_names;
    std::vector<double> m_values;
};

// ****************************************************************************
//! @brief Outcome of one assertion.
// ****************************************************************************
struct Check
{
    std::string text;
    bool passed = false;
    //!< Measured value, or why the assertion could not be evaluated.
    std::string detail;
};

//! @brief Values that are not stored in a @ref Metrics table (e.g. queries
//! with arguments such as @c object("a").inside("b")).
using MetricResolver = std::function<std::optional<double>(std::string_view)>;

// ----------------------------------------------------------------------------
//! @brief Evaluates @p_assertion: @c "<metric> <op> <number>" with @c op one
//! of @c == @c != @c < @c <= @c > @c >=, or a bare @c "<metric>" which passes
//! when the value is not zero. Unknown metrics fail with a detail.
// ----------------------------------------------------------------------------
[[nodiscard]] Check evaluate(std::string_view p_assertion,
                             Metrics const& p_metrics,
                             MetricResolver const& p_resolver = {});

} // namespace robotik
