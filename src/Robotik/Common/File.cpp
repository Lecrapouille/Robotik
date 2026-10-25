// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "Robotik/Common/File.hpp"

#include <algorithm>
#include <system_error>

namespace robotik
{

// -----------------------------------------------------------------------------
bool isDirectory(std::filesystem::path const& p_path)
{
    std::error_code error;
    return std::filesystem::is_directory(p_path, error);
}

// -----------------------------------------------------------------------------
bool isRegularFile(std::filesystem::path const& p_path)
{
    std::error_code error;
    return std::filesystem::is_regular_file(p_path, error);
}

// -----------------------------------------------------------------------------
std::filesystem::path canonicalPath(std::filesystem::path const& p_path)
{
    std::error_code error;
    std::filesystem::path result =
        std::filesystem::weakly_canonical(p_path, error);
    return error ? std::filesystem::absolute(p_path).lexically_normal()
                 : result;
}

// -----------------------------------------------------------------------------
bool pathInsideRoot(std::filesystem::path const& p_root,
                    std::filesystem::path const& p_file)
{
    std::filesystem::path const relative =
        canonicalPath(p_file).lexically_relative(canonicalPath(p_root));
    if (relative.empty())
    {
        return false;
    }
    return std::ranges::none_of(relative,
                                [](std::filesystem::path const& p_part)
                                { return p_part == ".."; });
}

// -----------------------------------------------------------------------------
std::filesystem::path executableDirectory()
{
#if defined(__EMSCRIPTEN__)
    // The page has no host path. Plugins are preloaded at /plugins.
    return "/";
#else
    std::error_code error;
    std::filesystem::path const exe =
        std::filesystem::read_symlink("/proc/self/exe", error);
    return error ? std::filesystem::path{} : exe.parent_path();
#endif
}

} // namespace robotik
