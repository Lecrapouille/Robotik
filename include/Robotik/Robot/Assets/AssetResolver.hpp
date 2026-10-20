// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file AssetResolver.hpp
/// Turns a model identifier into a file, whatever the current directory.
///
/// Packs are found under search roots, in this order:
/// @li each directory of @c ROBOTIK_DATA (colon separated);
/// @li @c data/packs next to the build folder of the running executable;
/// @li @c share/robotik/packs of the install prefix of the executable;
/// @li the @c data folder of each plugin package, added by the host.
///
/// A root is either a pack (it holds @c pack.yaml) or a folder of packs. The
/// first pack found for an id wins; a later pack with the same id and another
/// version is reported as a conflict.
///
/// @code
/// robotik::AssetResolver assets = robotik::AssetResolver::standard();
/// std::filesystem::path urdf = assets.model("robotik.tools.vacuum");
/// @endcode

#pragma once

#include "Robotik/Robot/Assets/AssetPack.hpp"

#include <filesystem>
#include <string>
#include <string_view>
#include <vector>

namespace robotik
{

// ****************************************************************************
/// Catalog of the asset packs visible to a run.
///
/// @code
/// robotik::AssetResolver assets = robotik::AssetResolver::standard();
/// assets.addRoot("plugins/PickAndPlace/data");
/// for (std::string const& error : assets.errors())
///     std::cerr << error << '\n';
/// auto urdf = assets.resolve("robotik.robots.abb.irb2400", scenario_folder);
/// @endcode
// ****************************************************************************
class AssetResolver
{
public:

    // -------------------------------------------------------------------------
    /// @return A resolver over the default roots (see the file description).
    // -------------------------------------------------------------------------
    [[nodiscard]] static AssetResolver standard();

    // -------------------------------------------------------------------------
    /// @return The default roots that exist on this machine, in search order.
    // -------------------------------------------------------------------------
    [[nodiscard]] static std::vector<std::filesystem::path> standardRoots();

    // -------------------------------------------------------------------------
    /// Loads the pack at a root, or every pack one level below it.
    /// Unreadable manifests are kept in @ref errors, not thrown.
    /// @param p_root A pack folder or a folder of packs.
    // -------------------------------------------------------------------------
    void addRoot(std::filesystem::path const& p_root);

    // -------------------------------------------------------------------------
    /// @return Every pack found, in search order.
    // -------------------------------------------------------------------------
    [[nodiscard]] std::vector<AssetPack> const& packs() const
    {
        return m_packs;
    }

    // -------------------------------------------------------------------------
    /// @return Manifest errors and version conflicts met while scanning.
    // -------------------------------------------------------------------------
    [[nodiscard]] std::vector<std::string> const& errors() const
    {
        return m_errors;
    }

    // -------------------------------------------------------------------------
    /// Finds a pack by its id.
    /// @param p_id Pack id (e.g. @c robotik.tools).
    /// @return The pack, or null.
    // -------------------------------------------------------------------------
    [[nodiscard]] AssetPack const* find(std::string_view p_id) const;

    // -------------------------------------------------------------------------
    /// Finds a pack and checks its version.
    /// @param p_id Pack id.
    /// @param p_minimum Oldest version accepted (same major version).
    /// @param p_why Receives the reason of a refusal.
    /// @return The pack when its version satisfies @p_minimum, null
    ///   otherwise.
    // -------------------------------------------------------------------------
    [[nodiscard]] AssetPack const* require(std::string_view p_id,
                                           Version const& p_minimum,
                                           std::string& p_why) const;

    // -------------------------------------------------------------------------
    /// Gives the file of a model. The longest pack id prefixing @p_id wins,
    /// so @c robotik.robots.abb.irb2400 picks the pack
    /// @c robotik.robots.abb over @c robotik.robots.
    /// @param p_id Model id, @c <pack-id>.<key>.
    /// @return Absolute file of the model.
    /// @throws std::runtime_error when no pack provides the model.
    // -------------------------------------------------------------------------
    [[nodiscard]] std::filesystem::path model(std::string_view p_id) const;

    // -------------------------------------------------------------------------
    /// Gives the file named by a scenario field. A reference holding a slash
    /// or ending in @c .urdf is a path; anything else is a model identifier
    /// (see @ref model).
    /// @param p_reference Path or model id written in the YAML.
    /// @param p_base Folder a relative path starts from.
    /// @return The file.
    /// @throws std::runtime_error when a model id is unknown.
    // -------------------------------------------------------------------------
    [[nodiscard]] std::filesystem::path
    resolve(std::string const& p_reference,
            std::filesystem::path const& p_base) const;

private:

    /// Loads one pack and checks it against the packs already known.
    void addPack(std::filesystem::path const& p_root);

    /// Packs found, in search order.
    std::vector<AssetPack> m_packs;
    /// Errors met while scanning.
    std::vector<std::string> m_errors;
};

} // namespace robotik
