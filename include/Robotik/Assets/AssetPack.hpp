// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

//! @file AssetPack.hpp
//! @brief A versioned folder of reusable assets (robots, tools, props).
//!
//! @code
//! # data/packs/robotik.tools/pack.yaml
//! id: robotik.tools
//! version: 1.0.0
//! license: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
//! source: https://github.com/Lecrapouille/RobotIK
//! models:
//!   vacuum: tool_vacuum.urdf
//!   gripper: tool_gripper.urdf
//! @endcode
//!
//! A model is named by the pack id and its key: @c robotik.tools.vacuum. The
//! files stay where the pack puts them; nobody outside the pack builds a path
//! to them.
#pragma once

#include "Robotik/Assets/Version.hpp"

#include <filesystem>
#include <map>
#include <string>

namespace robotik
{

// ****************************************************************************
//! @brief Manifest of one asset pack.
// ****************************************************************************
struct AssetPack
{
    //! @brief SPDX token for a license nobody has checked yet.
    static constexpr char const* UNVERIFIED_LICENSE = "NOASSERTION";

    // -------------------------------------------------------------------------
    //! @brief Reads @p_manifest (a @c pack.yaml).
    //! @throws std::runtime_error when a required field is missing, a model
    //! file does not exist or leaves the pack folder.
    // -------------------------------------------------------------------------
    [[nodiscard]] static AssetPack load(std::filesystem::path const& p_manifest);

    //! @brief Folder holding @c pack.yaml.
    std::filesystem::path root;
    std::string id;
    Version version;
    //!< SPDX expression, or @ref UNVERIFIED_LICENSE.
    std::string license;
    //!< Where the assets come from (URL), for the credits.
    std::string source;
    std::string description;
    //!< Model key to its file, relative to @ref root.
    std::map<std::string, std::filesystem::path> models;

    //! @brief False until somebody wrote down the license of the assets.
    [[nodiscard]] bool licenseVerified() const
    {
        return !license.empty() && license != UNVERIFIED_LICENSE;
    }

    //! @brief Absolute file of model @p_key, or empty when the pack has none.
    [[nodiscard]] std::filesystem::path model(std::string const& p_key) const;
};

} // namespace robotik
