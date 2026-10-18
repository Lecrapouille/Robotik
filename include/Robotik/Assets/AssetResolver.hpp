// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file AssetResolver.hpp
/// @brief Turns a model identifier into a file, whatever the current
/// directory.
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

#include "Robotik/Assets/AssetPack.hpp"

#include <filesystem>
#include <string>
#include <string_view>
#include <vector>

namespace robotik
{

/// @brief Folder of the running executable (Linux @c /proc/self/exe), or
/// empty. Search roots hang from it so that no lookup depends on the current
/// directory.
[[nodiscard]] std::filesystem::path executableDirectory();

// ****************************************************************************
/// @brief Catalog of the asset packs visible to a run.
// ****************************************************************************
class AssetResolver
{
public:

    /// @brief Resolver over the default roots (see the file description).
    [[nodiscard]] static AssetResolver standard();

    /// @brief Default roots that exist on this machine, in search order.
    [[nodiscard]] static std::vector<std::filesystem::path> standardRoots();

    // -------------------------------------------------------------------------
    /// @brief Loads the pack at @p_root, or every pack one level below it.
    /// Unreadable manifests are kept in @ref errors, not thrown.
    // -------------------------------------------------------------------------
    void addRoot(std::filesystem::path const& p_root);

    [[nodiscard]] std::vector<AssetPack> const& packs() const
    {
        return m_packs;
    }

    /// @brief Manifest errors and version conflicts met while scanning.
    [[nodiscard]] std::vector<std::string> const& errors() const
    {
        return m_errors;
    }

    [[nodiscard]] AssetPack const* find(std::string_view p_id) const;

    // -------------------------------------------------------------------------
    /// @brief Pack @p_id when its version satisfies @p_minimum, null
    /// otherwise. @p_why receives the reason of a refusal.
    // -------------------------------------------------------------------------
    [[nodiscard]] AssetPack const* require(std::string_view p_id,
                                           Version const& p_minimum,
                                           std::string& p_why) const;

    // -------------------------------------------------------------------------
    /// @brief File of model @p_id (@c <pack-id>.<key>). The longest pack id
    /// prefixing @p_id wins, so @c robotik.robots.abb.irb2400 picks the pack
    /// @c robotik.robots.abb over @c robotik.robots.
    /// @throws std::runtime_error when no pack provides the model.
    // -------------------------------------------------------------------------
    [[nodiscard]] std::filesystem::path model(std::string_view p_id) const;

    // -------------------------------------------------------------------------
    /// @brief File named by a scenario field. A reference holding a slash or
    /// ending in @c .urdf is a path relative to @p_base; anything else is a
    /// model identifier (see @ref model).
    // -------------------------------------------------------------------------
    [[nodiscard]] std::filesystem::path
    resolve(std::string const& p_reference,
            std::filesystem::path const& p_base) const;

private:

    void addPack(std::filesystem::path const& p_root);

    std::vector<AssetPack> m_packs;
    std::vector<std::string> m_errors;
};

} // namespace robotik
