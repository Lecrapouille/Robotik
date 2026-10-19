// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file Version.hpp
/// Semantic version of an asset pack or a plugin, and the rule used to check
/// a requirement against it.

#pragma once

#include <compare>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>

namespace robotik
{

// ****************************************************************************
/// @c major.minor.patch. Missing parts read as zero: "1.2" is 1.2.0.
///
/// Versions compare part by part (major first). A new major version breaks
/// compatibility; a new minor or patch version does not.
///
/// @code
/// auto const have = robotik::parseVersion("1.4.2");
/// auto const need = robotik::parseVersion("1.2");
/// bool ok = have && need && robotik::satisfies(*have, *need); // true
/// bool no = robotik::satisfies(robotik::Version{ 2, 0, 0 }, *need); // false
/// @endcode
// ****************************************************************************
struct Version
{
    std::uint32_t major = 0; ///< Changes when compatibility breaks.
    std::uint32_t minor = 0; ///< Changes when features are added.
    std::uint32_t patch = 0; ///< Changes for fixes only.

    /// Compares major, then minor, then patch.
    auto operator<=>(Version const&) const = default;
};

// -------------------------------------------------------------------------
/// Parses a version text.
/// @param p_text "1", "1.2" or "1.2.3".
/// @return The version, or nothing on any other text.
// -------------------------------------------------------------------------
[[nodiscard]] std::optional<Version> parseVersion(std::string_view p_text);

// -------------------------------------------------------------------------
/// @param p_version Version to format.
/// @return The version as text, always with three parts ("1.2.0").
// -------------------------------------------------------------------------
[[nodiscard]] std::string toString(Version const& p_version);

// -------------------------------------------------------------------------
/// Tells whether @p_have can replace @p_required: same major version, and
/// not older.
/// @param p_have Version of a pack or a plugin.
/// @param p_required Version asked by a plugin or a world.
/// @return True when the requirement is met.
// -------------------------------------------------------------------------
[[nodiscard]] inline bool satisfies(Version const& p_have,
                                    Version const& p_required)
{
    return p_have.major == p_required.major && p_have >= p_required;
}

} // namespace robotik
