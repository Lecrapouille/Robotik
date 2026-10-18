// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file PackagePath.hpp
/// @brief Files named inside a plugin package.
///
/// A world, scenario or experiment file names other files (a world, a
/// behavior tree, a trajectory, a URDF). Those names are relative to the
/// package root, the folder holding @c plugin.yaml, whatever the folder of
/// the file that names them:
///
/// @code
/// # plugins/HelloWorld/experiments/hello.experiment.yaml
/// cases:
///   - { id: wave, scenario: scenarios/wave.scenario.yaml }
/// @endcode
///
/// A name may hold @c .. and cross symbolic links, but the real file (the
/// canonical path) must be inside the package; an absolute name is refused.
/// A package is a closed folder; it can be moved or installed as a whole.
#pragma once

#include <filesystem>
#include <string>

namespace robotik
{

// ----------------------------------------------------------------------------
/// @brief Folder holding @c plugin.yaml: the folder of @p_file or one of its
/// parents.
/// @throws std::runtime_error when @p_file is in no package.
// ----------------------------------------------------------------------------
[[nodiscard]] std::filesystem::path packageRoot(std::filesystem::path const& p_file);

// ----------------------------------------------------------------------------
/// @brief Canonical path of @p_root / @p_name, or empty when @p_name is
/// empty.
/// @throws std::runtime_error when @p_name is absolute, or when its
/// canonical path is not inside the canonical @p_root.
// ----------------------------------------------------------------------------
[[nodiscard]] std::filesystem::path packageFile(std::filesystem::path const& p_root,
                                                std::string const& p_name);

} // namespace robotik
