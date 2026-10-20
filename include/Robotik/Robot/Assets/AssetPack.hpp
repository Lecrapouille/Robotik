/// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
/// Copyright (c) 2020-2026 Quentin Quadrat
///
/// This file is part of RobotIK. It is available under the GNU GPL v3 or,
/// for users who cannot use the GPL, under a commercial license.
/// See LICENSING.md for details.

/// @file AssetPack.hpp
/// A versioned folder of reusable assets (robots, tools, props) for RobotIK
/// plugins.
///
/// A model is named by the pack id and its key: @c robotik.tools.vacuum. The
/// files stay where the pack puts them; nobody outside the pack builds a path
/// to them. Example: data/packs/robotik.tools/pack.yaml.

#pragma once

#include "Robotik/Common/Result.hpp"
#include "Robotik/Common/Version.hpp"

#include <filesystem>
#include <map>
#include <string>

namespace robotik
{

// ****************************************************************************
/// Manifest of one asset pack.
///
/// Usually loaded by an @ref AssetResolver, which scans every pack folder.
/// Load it yourself only to inspect one pack.
///
/// # data/packs/robotik.tools/pack.yaml
/// @code
/// id: robotik.tools
/// version: 1.0.0
/// license: your license here
/// source: https://your.website.here
/// description: "Tools for RobotIK: vacuum, gripper, ..."
/// models:
///   vacuum: tool_vacuum.urdf
///   gripper: tool_gripper.urdf
/// @endcode
///
/// @code
/// auto const pack =
/// robotik::loadAssetPack("data/packs/robotik.tools/pack.yaml");
/// std::filesystem::path urdf = robotik::model(pack, "vacuum"); //
/// tool_vacuum.urdf if (!robotik::licenseVerified(pack)) {
///     std::cerr << pack.id << ": license not checked\n";
/// }
/// @endcode
// ****************************************************************************
struct AssetPack
{
    /// Folder holding @c pack.yaml.
    std::filesystem::path root;
    /// Pack id, the prefix of its model names (e.g. @c robotik.tools).
    std::string id;
    /// Version of the pack, checked against what a world requires.
    Version version;
    /// SPDX expression, or @c NOASSERTION when nobody checked the license.
    std::string license;
    /// Where the assets come from (URL), for the credits.
    std::string source;
    /// Free text shown in the credits.
    std::string description;
    /// Model key to its file, relative to @ref root.
    std::map<std::string, std::filesystem::path, std::less<>> models;
};

// -------------------------------------------------------------------------
/// Reads a pack manifest.
/// @param p_manifest Path of a @c pack.yaml.
/// @return The pack, with absolute paths; fails when a required field is
///   missing, or a model file does not exist or leaves the pack folder.
// -------------------------------------------------------------------------
[[nodiscard]] Result<AssetPack> loadAssetPack(std::filesystem::path const& p_manifest);

// -------------------------------------------------------------------------
/// Tells whether somebody wrote down the license of the assets.
/// @param p_pack Pack to inspect.
/// @return False when the license is empty or @c NOASSERTION.
// -------------------------------------------------------------------------
[[nodiscard]] bool licenseVerified(AssetPack const& p_pack);

// -------------------------------------------------------------------------
/// Gives the file of one model.
/// @param p_pack Pack that owns the model.
/// @param p_key Model key in the pack (e.g. @c vacuum, not the full name).
/// @return Absolute file of the model, or empty when the pack has none.
// -------------------------------------------------------------------------
[[nodiscard]] std::filesystem::path model(AssetPack const& p_pack,
                                          std::string const& p_key);

} // namespace robotik
