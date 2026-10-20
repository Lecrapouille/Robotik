// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#pragma once

#include <filesystem>
#include <string>

struct RenderTarget;

/// Saves the color of @p_target into the PNG file @p_path.
/// @return An empty string, or why it failed.
[[nodiscard]] std::string saveScreenshot(RenderTarget const& p_target,
                                         std::filesystem::path const& p_path);
