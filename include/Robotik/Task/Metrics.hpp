// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file Metrics.hpp
/// Named numbers published by a run.
///
/// A mission publishes what it measures (@c laps, @c cross_track.max...),
/// the cell adds @c time and @c controller.success. The conditions of a
/// scenario read them by name (Condition.hpp), without the library knowing
/// what they mean:
/// @code
/// controller.success
/// (< time 60)
/// (< cross_track.max 0.08)
/// @endcode
///
/// @par Contract
/// - Promises: an unknown metric is reported as such, never read as 0.
/// - Expects: metric names are free text without spaces or parentheses.
#pragma once

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

} // namespace robotik
