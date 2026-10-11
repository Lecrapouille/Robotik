// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

//! @file Version.hpp
//! @brief Semantic version of an asset pack or a plugin, and the rule used to
//! check a requirement against it.
#pragma once

#include <compare>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>

namespace robotik
{

// ****************************************************************************
//! @brief @c major.minor.patch. Missing parts read as zero: "1.2" is 1.2.0.
// ****************************************************************************
struct Version
{
    std::uint32_t major = 0;
    std::uint32_t minor = 0;
    std::uint32_t patch = 0;

    //! @brief Parses "1", "1.2" or "1.2.3". Nothing on any other text.
    [[nodiscard]] static std::optional<Version> parse(std::string_view p_text);

    [[nodiscard]] std::string toString() const;

    // -------------------------------------------------------------------------
    //! @brief True when this version can replace @p_required: same major
    //! version, and not older.
    // -------------------------------------------------------------------------
    [[nodiscard]] bool satisfies(Version const& p_required) const
    {
        return major == p_required.major && *this >= p_required;
    }

    auto operator<=>(Version const&) const = default;
};

} // namespace robotik
