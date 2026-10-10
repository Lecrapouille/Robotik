// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "SimFrame.hpp"

#include "App.hpp"
#include "Window.hpp"

#include <algorithm>
#include <imgui.h>

compages::world::ViewFrame viewFrame(App const& p_app,
                                     Window const& p_window,
                                     float p_elapsed,
                                     float p_total)
{
    ImGuiIO const& io = ImGui::GetIO();

    compages::world::ViewFrame frame;
    frame.width = p_app.view.width;
    frame.height = p_app.view.height;
    frame.elapsed = p_elapsed;
    frame.total = p_total;
    frame.input.mouse_over = p_app.view_hovered;

    if (p_app.view_hovered)
    {
        frame.input.mouse_delta =
            compages::core::Vector2f(io.MouseDelta.x, -io.MouseDelta.y);
        frame.input.scroll = std::clamp(p_window.scroll(), -1.0f, 1.0f);
        frame.input.mouse_right = io.MouseDown[ImGuiMouseButton_Right];
    }

    return frame;
}
