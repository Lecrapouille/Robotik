// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file Space.hpp
/// @brief Layout of an observation or an action vector: what each float is.
///
/// @code
/// robotik::Space observation;
/// observation.add3("tip", -1.0f, 1.0f, "m")       // tip.x, tip.y, tip.z
///            .add("holding", 0.0f, 1.0f)
///            .add("delivered", 0.0f, 1.0f, "", true); // from the ground truth
/// @endcode
///
/// A dimension marked @c privileged is computed from the ground truth of the
/// simulator: a real robot could not observe it. A policy trained on it must
/// not be taken for a deployable one.
#pragma once

#include <cstddef>
#include <limits>
#include <span>
#include <string>
#include <vector>

namespace robotik
{

// ****************************************************************************
/// @brief Named, bounded dimensions of a flat float vector.
// ****************************************************************************
struct Space
{
    struct Dimension
    {
        std::string name;
        float low = -std::numeric_limits<float>::infinity();
        float high = std::numeric_limits<float>::infinity();
        /// SI unit, empty for a flag or a ratio.
        std::string unit;
        /// Comes from the ground truth, not from a sensor.
        bool privileged = false;
    };

    std::vector<Dimension> dimensions;

    /// @brief @p_count unbounded dimensions named @p_prefix0, @p_prefix1...
    [[nodiscard]] static Space unnamed(std::string const& p_prefix, std::size_t p_count)
    {
        Space space;
        for (std::size_t i = 0; i < p_count; ++i)
        {
            Dimension dimension;
            dimension.name = p_prefix + std::to_string(i);
            space.dimensions.push_back(std::move(dimension));
        }
        return space;
    }

    Space& add(std::string p_name,
               float p_low,
               float p_high,
               std::string p_unit = {},
               bool p_privileged = false)
    {
        dimensions.push_back(
            { std::move(p_name), p_low, p_high, std::move(p_unit), p_privileged });
        return *this;
    }

    /// @brief Three dimensions @p_name.x, .y and .z with the same bounds.
    Space& add3(std::string const& p_name,
                float p_low,
                float p_high,
                std::string const& p_unit = {},
                bool p_privileged = false)
    {
        for (char const* axis : { ".x", ".y", ".z" })
        {
            add(p_name + axis, p_low, p_high, p_unit, p_privileged);
        }
        return *this;
    }

    [[nodiscard]] std::size_t size() const
    {
        return dimensions.size();
    }

    /// @brief One dimension or more comes from the ground truth.
    [[nodiscard]] bool privileged() const
    {
        for (Dimension const& dimension : dimensions)
        {
            if (dimension.privileged)
            {
                return true;
            }
        }
        return false;
    }

    /// @brief Every value lies within its bounds (and is not NaN).
    [[nodiscard]] bool contains(std::span<float const> p_values) const
    {
        if (p_values.size() != dimensions.size())
        {
            return false;
        }
        for (std::size_t i = 0; i < p_values.size(); ++i)
        {
            if (!(p_values[i] >= dimensions[i].low && p_values[i] <= dimensions[i].high))
            {
                return false;
            }
        }
        return true;
    }
};

} // namespace robotik
