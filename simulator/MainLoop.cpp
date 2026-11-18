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

#include "Compages/GPU/Errors.hpp"
#include "Compages/GPU/RenderPass.hpp"
#include "Compages/Renderer/Animation/AnimationSystem.hpp"
#include "Compages/World/Controllers/ViewFrame.hpp"

#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

#include <algorithm>
#include <iostream>

#if defined(__EMSCRIPTEN__) && !defined(EXAEQUOS)
#    include <emscripten.h>
#endif

#include "Browser.hpp"

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
        // The run goes on whatever the panels show: the 3D view is only
        // drawn when it has a picture to draw into.
        if (p_window.device_ready && p_app.simulation)
        {
            // Two clocks: the routines see the time the simulation moved
            // (zero while paused), the camera rig the time of the wall.
            double const before = p_app.simulation->time().value();
            p_app.advance(elapsed);
            compages::si::Seconds simulated(0.0);
            compages::si::Seconds now(0.0);
            if (p_app.simulation)
            {
                now = compages::si::Seconds(p_app.simulation->time().value());
                simulated = compages::si::Seconds(std::max(0.0, now.value() - before));
            }
            p_app.world->update(viewFrame(p_app, p_window, simulated, compages::si::Seconds(elapsed), now));
            // The clips and the skins of the models, with the same time.
            compages::gpu::check(compages::renderer::AnimationSystem::tick(
                *p_app.world, p_app.scene->assets(), simulated));
            if (p_app.view.valid())
            {
                p_app.scene->render(p_app.view_camera, p_app.view);
            }
        }
        // --- Model of the Asset tab of Files, in its own world ---
        AssetPreview& preview = p_app.ui.asset_preview;
        if (ModelView* model = preview.model.get();
            model != nullptr && preview.visible && !preview.show_text &&
            model->target.valid())
        {
            // Nothing pauses a preview: its two clocks are the wall.
            model->world->update(viewFrame(model->target, model->area, p_window,
                                           compages::si::Seconds(elapsed), compages::si::Seconds(elapsed),
                                           compages::si::Seconds(total)));
            model->scene->render(model->camera, model->target);
            model->rendered = true;
        }
        // --screenshot: the 3D view at the asked time, then quit.
        if (!p_app.screenshot.empty() && p_app.simulation &&
            p_app.view.valid() &&
            (p_app.simulation->time().value() >= p_app.screenshot_at ||
             p_app.simulation->finished()))
        {
            compages::Status const saved = p_app.view.saveColor(p_app.screenshot);
            std::string const failed = saved ? std::string() : saved.error();
            std::cerr << (failed.empty() ? "wrote " + p_app.screenshot.string()
                                         : failed)
                      << '\n';
            if (failed.empty())
            {
                robotik::download(p_app.screenshot.string());
            }
            p_app.screenshot.clear();
            glfwSetWindowShouldClose(p_window.handle, GLFW_TRUE);
        }

        ImGui::Render();

        // --- Full-window clear and ImGui draw on top ---
        int width = 0;
        int height = 0;
        glfwGetFramebufferSize(p_window.handle, &width, &height);
        if (p_window.device_ready)
        {
            compages::gpu::RenderPass pass(compages::gpu::PassDesc{
                .width = static_cast<std::uint32_t>(width),
                .height = static_cast<std::uint32_t>(height),
                .color = { 0.05F, 0.05F, 0.06F, 1.0F } });
        }
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        // --screenshot-ui: the whole window at the asked time, then quit.
        // A few frames first, so the docks have their sizes.
        if (!p_app.screenshot_ui.empty() && ImGui::GetFrameCount() > 30 &&
            (!p_app.simulation ||
             p_app.simulation->time().value() >= p_app.screenshot_at ||
             p_app.simulation->finished()))
        {
            bool const saved = saveWindowPng(p_app.screenshot_ui, width, height);
            std::cerr << (saved ? "wrote " : "could not write ")
                      << p_app.screenshot_ui.string() << '\n';
            p_app.screenshot_ui.clear();
            glfwSetWindowShouldClose(p_window.handle, GLFW_TRUE);
        }
        glfwSwapBuffers(p_window.handle);

#if defined(__EMSCRIPTEN__) && !defined(EXAEQUOS)
        emscripten_sleep(0);
#endif
    }
}
