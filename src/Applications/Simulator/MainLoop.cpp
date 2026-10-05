// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "MainLoop.hpp"

#include "App.hpp"
#include "SimFrame.hpp"
#include "SimulatorDisplay.hpp"
#include "Window.hpp"

#include "Compages/GPU/RenderPass.hpp"
#include "Compages/World/Controllers/ViewFrame.hpp"

#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

void runMainLoop(Window& p_window, App& p_app)
{
    double last = glfwGetTime();
    float total = 0.0f;

    while (glfwWindowShouldClose(p_window.handle) == GLFW_FALSE)
    {
        // --- Input and frame timing ---
        glfwPollEvents();
        double const now = glfwGetTime();
        float const elapsed = static_cast<float>(now - last);
        last = now;
        total += elapsed;

        // --- Operator UI (dock, panels, play/pause) ---
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();
        drawPanels(p_app);
        ImGui::Render();

        // --- World simulation and 3D views (when scenario loaded) ---
        if (p_app.simulation && p_app.view.width > 0)
        {
            p_app.advance(elapsed);
            p_app.world->update(viewFrame(p_app, elapsed, total));
            compages::gpu::RenderPass pass(
                p_app.view.framebuffer,
                compages::gpu::PassDesc{ .color = viewClearColor() });
            p_app.scene->render(p_app.view_camera);
        }

        // --- Full-window clear and ImGui draw on top ---
        int width = 0;
        int height = 0;
        glfwGetFramebufferSize(p_window.handle, &width, &height);
        {
            compages::gpu::RenderPass pass(compages::gpu::PassDesc{
                .width = static_cast<std::uint32_t>(width),
                .height = static_cast<std::uint32_t>(height),
                .color = { 0.05f, 0.05f, 0.06f, 1.0f } });
        }
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(p_window.handle);
    }
}
