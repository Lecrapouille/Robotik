#include "SimFrame.hpp"

#include "App.hpp"
#include "SimulatorDisplay.hpp"

#include "Robotik/ECS/PerceptionComponents.hpp"

#include "Compages/GPU/RenderPass.hpp"

#include <imgui.h>

#include <cstdint>
#include <span>

//------------------------------------------------------------------------------
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

//------------------------------------------------------------------------------
void perceive(App& p_app)
{
    // Get the camera entity from the simulation
    compages::world::Entity camera = p_app.simulation->camera();
    if (!camera)
    {
        return;
    }

    // Get the camera sensor from the camera entity
    auto const& sensor = camera.get<robotik::ecs::CameraSensor>();
    if (!p_app.robot_view.resize(sensor.width, sensor.height))
    {
        return;
    }

    // Render the camera sensor to the robot view
    {
        compages::gpu::RenderPass pass(
            p_app.robot_view.framebuffer,
            compages::gpu::PassDesc{ .color = viewClearColor() });
        p_app.scene->render(camera);
    }

    // Read the pixels from the robot view
    auto pixels = p_app.robot_view.color.read();
    if (!pixels)
    {
        return;
    }

    // Detect objects in the robot view
    std::span<const std::byte> const bytes = pixels.value();
    auto& detected = camera.get<robotik::ecs::DetectedObjects>();

    // Detect objects in the robot view
    detected.items = p_app.detector.detect(
        { reinterpret_cast<std::uint8_t const*>(bytes.data()), bytes.size() },
        static_cast<int>(sensor.width),
        static_cast<int>(sensor.height));
    ++detected.frame;
}
