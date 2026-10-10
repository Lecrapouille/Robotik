// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#pragma once

// Generated file holding project information
#include "project_info.hpp"

// Google Test and Google Mock frameworks
#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <filesystem>

using namespace ::testing;

//! @brief Repository file, whether tests run from the root or from tests/.
inline std::filesystem::path repoFile(std::filesystem::path const& p_file)
{
    for (char const* root : { ".", ".." })
    {
        std::filesystem::path const candidate = std::filesystem::path(root) / p_file;
        if (std::filesystem::exists(candidate))
        {
            return candidate;
        }
    }
    return p_file;
}

//! @brief File of the data folder, whether tests run from the root or tests/.
inline std::filesystem::path dataFile(std::filesystem::path const& p_file)
{
    for (char const* root : { "data", "../data" })
    {
        if (std::filesystem::exists(std::filesystem::path(root) / p_file))
        {
            return std::filesystem::path(root) / p_file;
        }
    }
    return std::filesystem::path("data") / p_file;
}
