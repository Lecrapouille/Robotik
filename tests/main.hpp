// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#pragma once

// Generated file holding project information
#include "robotik-core/project_info.hpp"

// Google Test and Google Mock frameworks
#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "Robotik/Robot/Assets/AssetResolver.hpp"

#include <filesystem>
#include <string_view>

using namespace ::testing;

/// Repository file, whether tests run from the root or from tests/.
inline std::filesystem::path repoFile(std::filesystem::path const& p_file)
{
    for (char const* root : { ".", ".." })
    {
        std::filesystem::path const candidate =
            std::filesystem::path(root) / p_file;
        if (std::filesystem::exists(candidate))
        {
            return candidate;
        }
    }
    return p_file;
}

/// Asset packs of the repository: data/packs and the plugins' own.
inline robotik::AssetResolver const& testAssets()
{
    static robotik::AssetResolver const assets = []()
    {
        robotik::AssetResolver resolver = robotik::AssetResolver::standard();
        for (auto const& entry :
             std::filesystem::directory_iterator(repoFile("plugins")))
        {
            resolver.addRoot(entry.path() / "data");
        }
        return resolver;
    }();
    return assets;
}

/// File of the model @p_id of the repository packs.
inline std::filesystem::path assetFile(std::string_view p_id)
{
    return testAssets().model(p_id).take();
}
