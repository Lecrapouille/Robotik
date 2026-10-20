// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file PackagePath.hpp
/// Files named inside a plugin package.
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
///
/// @par Contract
/// - Promises: @c packageFile returns a @c Result — a path that leaves the
///   package, or an absolute path, is refused with a message.
#pragma once

#include "Robotik/Common/Result.hpp"

#include <filesystem>
#include <string>

namespace robotik
{

// ----------------------------------------------------------------------------
/// Finds the package that holds a file.
///
/// @code
/// packageRoot("plugins/HelloWorld/scenarios/wave.scenario.yaml");
/// // -> plugins/HelloWorld
/// @endcode
///
/// @param p_file Any file of the package.
/// @return Folder holding @c plugin.yaml: the folder of @p_file or one of
///   its parents.
///   Fails when @p_file is in no package.
// ----------------------------------------------------------------------------
[[nodiscard]] Result<std::filesystem::path>
packageRoot(std::filesystem::path const& p_file);

// ----------------------------------------------------------------------------
/// Resolves a file name written in a YAML file of the package.
///
/// @code
/// auto root = packageRoot(scenario_file);
/// if (root)
///     auto world = packageFile(root.value(), "worlds/arm.world.yaml");
/// @endcode
///
/// @param p_root Package root, as given by @ref packageRoot.
/// @param p_name Name relative to @p_root.
/// @return Canonical path of @p_root / @p_name, or empty when @p_name is
///   empty.
///   Fails when @p_name is absolute, or when its canonical path is not
///   inside the canonical @p_root.
// ----------------------------------------------------------------------------
[[nodiscard]] Result<std::filesystem::path>
packageFile(std::filesystem::path const& p_root, std::string const& p_name);

} // namespace robotik
