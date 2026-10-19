// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "Robotik/Common/Version.hpp"

#include <array>
#include <charconv>

namespace robotik
{

// -----------------------------------------------------------------------------
std::optional<Version> parseVersion(std::string_view p_text)
{
    Version version;
    std::array<std::uint32_t*, 3> const parts = { &version.major,
                                                  &version.minor,
                                                  &version.patch };
    std::size_t index = 0;
    char const* cursor = p_text.data();
    char const* const end = p_text.data() + p_text.size();
    while (true)
    {
        if (index == 3u || cursor == end)
        {
            return std::nullopt;
        }
        auto const [next, error] = std::from_chars(cursor, end, *parts[index]);
        if (error != std::errc{})
        {
            return std::nullopt;
        }
        cursor = next;
        ++index;
        if (cursor == end)
        {
            return version;
        }
        if (*cursor != '.')
        {
            return std::nullopt;
        }
        ++cursor;
    }
}

// -----------------------------------------------------------------------------
std::string toString(Version const& p_version)
{
    return std::to_string(p_version.major) + "." +
           std::to_string(p_version.minor) + "." +
           std::to_string(p_version.patch);
}

} // namespace robotik
