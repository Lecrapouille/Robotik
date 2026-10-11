// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

//! @file Metrics.hpp
//! @brief Named numbers published by a run, and the conditions evaluated on
//! them.
//!
//! A mission publishes what it measures (@c laps, @c cross_track.max...),
//! the simulation adds @c time and @c controller.success, and answers queries
//! on the world such as @c inside(red_cube, box). A condition compares one of
//! them to a number, without the library knowing what they mean:
//! @code
//! controller.success
//! time < 60
//! cross_track.max < 0.08
//! pose_error(easel) < 0.002
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

//! @brief Values that are not stored in a @ref Metrics table (e.g. queries
//! with arguments such as @c inside(red_cube, box)). Throws
//! std::invalid_argument for a query it knows but cannot answer (unknown
//! entity), returns nothing for a name it does not know.
using MetricResolver = std::function<std::optional<double>(std::string_view)>;

// ****************************************************************************
//! @brief One evaluation of a condition.
// ****************************************************************************
struct Evaluation
{
    //!< False when the value could not be obtained (see @ref detail).
    bool known = false;
    //!< The condition is true.
    bool holds = false;
    //!< Measured value of the left-hand side.
    double value = 0.0;
    //!< Right-hand side, e.g. "< 0.002", or "!= 0" for a bare value.
    std::string expected;
    //!< "name = value", or why nothing was measured.
    std::string detail;
};

// ----------------------------------------------------------------------------
//! @brief Evaluates @p_condition: @c "<value> <op> <number>" with @c op one of
//! @c == @c != @c < @c <= @c > @c >=, or a bare @c "<value>" which holds when
//! the value is not zero. A value is a metric or a resolver query.
// ----------------------------------------------------------------------------
[[nodiscard]] Evaluation evaluate(std::string_view p_condition,
                                  Metrics const& p_metrics,
                                  MetricResolver const& p_resolver = {});

} // namespace robotik
