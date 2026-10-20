// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file Metrics.hpp
/// Named numbers published by a run, and the conditions evaluated on them.
///
/// A mission publishes what it measures (@c laps, @c cross_track.max...),
/// the simulation adds @c time and @c controller.success, and answers queries
/// on the world such as @c inside(red_cube, box). A condition compares one of
/// them to a number, without the library knowing what they mean:
/// @code
/// controller.success
/// time < 60
/// cross_track.max < 0.08
/// pose_error(easel) < 0.002
/// @endcode
///
/// @par Contract
/// - Promises: an unknown metric is reported as such, never read as 0.
/// - Expects: metric names are free text without spaces.
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
/// Flat table of named values (insertion order kept).
///
/// @code
/// robotik::Metrics metrics;
/// metrics.set("laps", 3.0);
/// metrics.set("laps", 4.0);                 // overwrites
/// double laps = metrics.get("laps").value(); // 4
/// @endcode
// ****************************************************************************
class Metrics
{
public:

    // ------------------------------------------------------------------------
    /// Creates or overwrites a value.
    /// @param p_name Metric name; dots group names (@c cross_track.max).
    /// @param p_value Measured value.
    // ------------------------------------------------------------------------
    void set(std::string_view p_name, double p_value);

    // ------------------------------------------------------------------------
    /// Reads a value.
    /// @param p_name Metric name.
    /// @return The value, or nothing when it was never set.
    // ------------------------------------------------------------------------
    [[nodiscard]] std::optional<double> get(std::string_view p_name) const;

    // ------------------------------------------------------------------------
    /// Removes every value.
    // ------------------------------------------------------------------------
    void clear()
    {
        m_names.clear();
        m_values.clear();
    }

    // ------------------------------------------------------------------------
    /// @return The names, in insertion order, aligned with @ref values.
    // ------------------------------------------------------------------------
    [[nodiscard]] std::span<std::string const> names() const
    {
        return m_names;
    }

    // ------------------------------------------------------------------------
    /// @return The values, aligned with @ref names.
    // ------------------------------------------------------------------------
    [[nodiscard]] std::span<double const> values() const
    {
        return m_values;
    }

private:

    /// Metric names, in insertion order.
    std::vector<std::string> m_names;
    /// Values, aligned with @ref m_names.
    std::vector<double> m_values;
};

/// Values that are not stored in a @ref Metrics table (e.g. queries with
/// arguments such as @c inside(red_cube, box)). Throws std::invalid_argument
/// for a query it knows but cannot answer (unknown entity), returns nothing
/// for a name it does not know.
using MetricResolver = std::function<std::optional<double>(std::string_view)>;

// ****************************************************************************
/// One evaluation of a condition.
///
/// @code
/// robotik::Evaluation const e = robotik::evaluate("time < 60", metrics);
/// if (!e.known)
///     std::cerr << e.detail << '\n';   // why it could not be measured
/// else if (!e.holds)
///     std::cerr << e.detail << ", expected " << e.expected << '\n';
/// @endcode
// ****************************************************************************
struct Evaluation
{
    /// False when the value could not be obtained (see @ref detail).
    bool known = false;
    /// The condition is true.
    bool holds = false;
    /// Measured value of the left-hand side.
    double value = 0.0;
    /// Right-hand side, e.g. "< 0.002", or "!= 0" for a bare value.
    std::string expected;
    /// "name = value", or why nothing was measured.
    std::string detail;
};

// ----------------------------------------------------------------------------
/// Evaluates a condition written in a scenario.
///
/// The condition is @c "<value> <op> <number>" with @c op one of @c ==
/// @c != @c < @c <= @c > @c >=, or a bare @c "<value>" which holds when the
/// value is not zero. A value is a metric or a resolver query.
///
/// @param p_condition Text of the condition, e.g. @c "time < 60".
/// @param p_metrics Published metrics.
/// @param p_resolver Answers the values that are not in @p_metrics.
/// @return What was measured and whether the condition holds.
// ----------------------------------------------------------------------------
[[nodiscard]] Evaluation evaluate(std::string_view p_condition,
                                  Metrics const& p_metrics,
                                  MetricResolver const& p_resolver = {});

} // namespace robotik
