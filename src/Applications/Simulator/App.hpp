#pragma once

#include "Robotik/Perception/ColorDetector.hpp"
#include "Robotik/Runtime/Simulation.hpp"

#include "Compages/GPU/Framebuffer.hpp"
#include "Compages/GPU/Texture.hpp"
#include "Compages/Renderer/Scene.hpp"
#include "Compages/World/World.hpp"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

// Offscreen picture shown in an ImGui panel.
struct RenderTarget
{
    compages::gpu::Texture color;
    compages::gpu::Texture depth;
    compages::gpu::Framebuffer framebuffer;
    std::uint32_t width = 0;
    std::uint32_t height = 0;

    bool resize(std::uint32_t p_width, std::uint32_t p_height);
};

// Everything the operator interface shows and drives. The whole world is
// rebuilt from the scenario file on load and on reset.
struct App
{
    std::string scenario_path;
    std::string error;

    std::unique_ptr<compages::world::World> world;
    std::unique_ptr<compages::renderer::Scene> scene;
    std::unique_ptr<robotik::Simulation> simulation;
    compages::world::Entity view_camera;
    robotik::ColorDetector detector;

    RenderTarget view;
    RenderTarget robot_view;
    bool view_hovered = false;

    bool playing = true;
    bool step_once = false;
    float speed = 1.0f;

    void load();
};

// The dock layout and every panel, drawn each frame.
void drawPanels(App& p_app);
