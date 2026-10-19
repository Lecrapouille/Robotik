// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "Robotik/Scenario/PackagePath.hpp"

#include <algorithm>
#include <stdexcept>
#include <system_error>

namespace robotik
{

std::filesystem::path packageRoot(std::filesystem::path const& p_file)
{
    std::filesystem::path const file =
        std::filesystem::absolute(p_file).lexically_normal();
    for (std::filesystem::path folder = file.parent_path(); !folder.empty();
         folder = folder.parent_path())
    {
        std::error_code error;
        if (std::filesystem::is_regular_file(folder / "plugin.yaml", error))
        {
            return folder;
        }
        if (folder == folder.root_path())
        {
            break;
        }
    }
    throw std::runtime_error(
        p_file.string() + " is in no plugin package (no plugin.yaml above it)");
}

std::filesystem::path packageFile(std::filesystem::path const& p_root,
                                  std::string const& p_name)
{
    if (p_name.empty())
    {
        return {};
    }
    std::filesystem::path const name(p_name);
    if (name.is_absolute() || name.has_root_name() || name.has_root_directory())
    {
        throw std::runtime_error("'" + p_name +
                                 "' must be relative to the package root");
    }
    // ".." and symbolic links are followed: only the real file must be in
    // the package.
    std::error_code root_error;
    std::error_code file_error;
    std::filesystem::path const root =
        std::filesystem::weakly_canonical(p_root, root_error);
    std::filesystem::path const file =
        std::filesystem::weakly_canonical(p_root / name, file_error);
    if (root_error || file_error)
    {
        throw std::runtime_error(
            "'" + p_name +
            "': " + (root_error ? root_error : file_error).message());
    }
    auto const mismatch =
        std::mismatch(root.begin(), root.end(), file.begin(), file.end());
    if (mismatch.first != root.end() || file == root)
    {
        throw std::runtime_error("'" + p_name + "' leaves the package (" +
                                 file.string() + ")");
    }
    return file;
}

} // namespace robotik
