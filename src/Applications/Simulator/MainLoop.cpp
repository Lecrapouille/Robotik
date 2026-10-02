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
            bool const advance = p_app.playing || p_app.step_once;
            p_app.step_once = false;
            compages::world::ViewFrame frame =
                viewFrame(p_app, advance ? elapsed * p_app.speed : 0.0f, total);
            if (advance)
            {
                p_app.simulation->step(frame);
            }
            else
            {
                p_app.world->update(frame);
            }
            perceive(p_app);
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
