// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "SimFrame.hpp"

#include "App.hpp"

#include <imgui.h>

compages::world::ViewFrame
viewFrame(App const& p_app, float p_elapsed, float p_total)
{
    ImGuiIO const& io = ImGui::GetIO();

    // Initialize the view frame
    compages::world::ViewFrame frame;
    frame.width = p_app.view.width;
    frame.height = p_app.view.height;
    frame.elapsed = p_elapsed;
    frame.total = p_total;
    frame.input.mouse_over = p_app.view_hovered;

    // Set the mouse input if the view is hovered
    if (p_app.view_hovered)
    {
        frame.input.mouse_delta =
            compages::core::Vector2f(io.MouseDelta.x, -io.MouseDelta.y);
        frame.input.scroll = io.MouseWheel;
        frame.input.mouse_right = io.MouseDown[ImGuiMouseButton_Right];
    }

    return frame;
}
