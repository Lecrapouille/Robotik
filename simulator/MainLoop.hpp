// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#pragma once

#include <filesystem>

struct App;
struct Window;

// Event poll, simulation step, 3D views, ImGui composite, swap buffers.
void runMainLoop(Window& p_window, App& p_app);

/// Saves the window, panels included, as a PNG (--screenshot-ui). Call it
/// after the panels are drawn and before the buffers are swapped.
/// @return False when it could not read or write.
bool saveWindowPng(std::filesystem::path const& p_path, int p_width, int p_height);
