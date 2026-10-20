// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "MainLoop.hpp"

#include "Screenshot.hpp"

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

#include <iostream>

void runMainLoop(Window& p_window, App& p_app)
{
    double last = glfwGetTime();
    float total = 0.0F;

    while (glfwWindowShouldClose(p_window.handle) == GLFW_FALSE)
    {
        // --- Input and frame timing ---
        p_window.beginInputFrame();
        glfwPollEvents();
        if (!ImGui::GetIO().WantTextInput &&
            ImGui::IsKeyPressed(ImGuiKey_H, false))
        {
            p_app.plugins.publishKey('H');
        }
        double const now = glfwGetTime();
        float const elapsed = static_cast<float>(now - last);
        last = now;
        total += elapsed;

        // --- Operator UI (dock, panels, play/pause) ---
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();
        drawPanels(p_app);

        // --- World simulation and 3D views (when scenario loaded) ---
        if (p_app.simulation && p_app.view.width > 0)
        {
            p_app.advance(elapsed);
            p_app.world->update(viewFrame(p_app, p_window, elapsed, total));
            compages::gpu::RenderPass pass(
                p_app.view.framebuffer,
                compages::gpu::PassDesc{ .color = viewClearColor() });
            p_app.scene->render(p_app.view_camera);
        }
        // --- Model of the Asset tab of Files, in its own world ---
        AssetPreview& preview = p_app.ui.asset_preview;
        if (ModelView* model = preview.model.get();
            model != nullptr && preview.visible && !preview.show_text &&
            model->target.width > 0)
        {
            model->world->update(viewFrame(
                model->target, model->area, p_window, elapsed, total));
            compages::gpu::RenderPass pass(
                model->target.framebuffer,
                compages::gpu::PassDesc{ .color = viewClearColor() });
            model->scene->render(model->camera);
            model->rendered = true;
        }
        // --screenshot: the 3D view at the asked time, then quit.
        if (!p_app.screenshot.empty() && p_app.simulation &&
            p_app.view.width > 0 &&
            (p_app.simulation->time().value() >= p_app.screenshot_at ||
             p_app.simulation->finished()))
        {
            std::string const failed =
                saveScreenshot(p_app.view, p_app.screenshot);
            std::cerr << (failed.empty() ? "wrote " + p_app.screenshot.string()
                                         : failed)
                      << '\n';
            p_app.screenshot.clear();
            glfwSetWindowShouldClose(p_window.handle, GLFW_TRUE);
        }

        ImGui::Render();

        // --- Full-window clear and ImGui draw on top ---
        int width = 0;
        int height = 0;
        glfwGetFramebufferSize(p_window.handle, &width, &height);
        {
            compages::gpu::RenderPass pass(compages::gpu::PassDesc{
                .width = static_cast<std::uint32_t>(width),
                .height = static_cast<std::uint32_t>(height),
                .color = { 0.05F, 0.05F, 0.06F, 1.0F } });
        }
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(p_window.handle);
    }
}
