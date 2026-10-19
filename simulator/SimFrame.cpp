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
#include <utility>

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
        frame.input.mouse_left = io.MouseDown[ImGuiMouseButton_Left];
        frame.input.mouse_right = io.MouseDown[ImGuiMouseButton_Right];
        frame.input.mouse_middle = io.MouseDown[ImGuiMouseButton_Middle];
        // In the pixels of the frame, origin bottom-left.
        float const sx =
            static_cast<float>(p_app.view.width) / std::max(p_app.view_w, 1.0f);
        float const sy = static_cast<float>(p_app.view.height) /
                         std::max(p_app.view_h, 1.0f);
        frame.input.mouse = compages::core::Vector2f(
            (io.MousePos.x - p_app.view_x) * sx,
            (p_app.view_y + p_app.view_h - io.MousePos.y) * sy);
        // The keys move the camera only over the view, not while a text
        // field takes them, and not with Ctrl (Ctrl+R replays).
        if (!io.WantTextInput && !io.KeyCtrl)
        {
            using compages::world::Key;
            constexpr std::pair<Key, ImGuiKey> KEYS[] = {
                { Key::W, ImGuiKey_W },
                { Key::A, ImGuiKey_A },
                { Key::S, ImGuiKey_S },
                { Key::D, ImGuiKey_D },
                { Key::Q, ImGuiKey_Q },
                { Key::E, ImGuiKey_E },
                { Key::R, ImGuiKey_R },
                { Key::F, ImGuiKey_F },
                { Key::Up, ImGuiKey_UpArrow },
                { Key::Down, ImGuiKey_DownArrow },
                { Key::Left, ImGuiKey_LeftArrow },
                { Key::Right, ImGuiKey_RightArrow },
                { Key::Shift, ImGuiKey_LeftShift },
            };
            for (auto const& [key, imgui] : KEYS)
            {
                frame.input.set(key, ImGui::IsKeyDown(imgui));
            }
            frame.input.set(Key::Shift,
                            ImGui::IsKeyDown(ImGuiKey_LeftShift) ||
                                ImGui::IsKeyDown(ImGuiKey_RightShift));
        }
    }

    return frame;
}
