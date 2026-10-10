// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#pragma once

struct GLFWwindow;

// ****************************************************************************
//! @brief GLFW window, OpenGL 4.5 via Compages GPU, and Dear ImGui (docking).
// ****************************************************************************
struct Window
{
    GLFWwindow* handle = nullptr;
    bool glfw_ready = false;
    bool device_ready = false;
    bool imgui_ready = false;

    bool open();
    ~Window();

    //! @brief Scroll accumulated since the last beginInputFrame() (GLFW y offset).
    [[nodiscard]] float scroll() const
    {
        return m_scroll;
    }

    //! @brief Call once per frame after glfwPollEvents().
    void beginInputFrame();

private:

    static void onScroll(GLFWwindow* p_window, double p_x, double p_y);

    float m_scroll = 0.0f;
};
