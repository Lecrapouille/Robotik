// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "App.hpp"

#include "SimulatorDisplay.hpp"

#include "Robotik/Scenario/Scenario.hpp"

#include "Compages/Core/Units.hpp"
#include "Compages/World/Controllers/Controls.hpp"

#include <iostream>

bool RenderTarget::resize(std::uint32_t p_width, std::uint32_t p_height)
{
    if (p_width == width && p_height == height)
    {
        return width != 0;
    }
    width = 0;
    height = 0;
    compages::Status made =
        color.allocate({ .format = compages::gpu::PixelFormat::RGB8,
                         .width = p_width,
                         .height = p_height });
    if (made)
    {
        made = depth.allocate({ .format = compages::gpu::PixelFormat::Depth32F,
                                .width = p_width,
                                .height = p_height,
                                .magnify = compages::gpu::Filter::Nearest,
                                .minify = compages::gpu::Filter::Nearest });
    }
    if (made)
    {
        made = framebuffer.attach(color, depth);
    }
    if (!made)
    {
        std::cerr << "Render target: " << made.error() << '\n';
        return false;
    }
    width = p_width;
    height = p_height;
    return true;
}

void App::load()
{
    error.clear();
    simulation.reset();
    scene.reset();
    world = std::make_unique<compages::world::World>();
    scene = std::make_unique<compages::renderer::Scene>(*world);
    auto const clear = viewClearColor();
    scene->background(clear.x, clear.y, clear.z).ambient(0.35f, 0.35f, 0.38f);
    scene->sun("Sun")
        .rotation(Radians(-0.9f), compages::core::Vector3f(1.0f, 0.3f, 0.0f));
    view_camera = scene->camera("ViewCamera");
    scene->activeCamera(view_camera);
    detector = {};
    try
    {
        simulation = std::make_unique<robotik::Simulation>(
            *world, scene.get(), robotik::Scenario::load(scenario_path));
    }
    catch (std::exception const& failure)
    {
        error = failure.what();
        simulation.reset();
        return;
    }

    scene->plane("Floor", compages::renderer::color(0.32f, 0.33f, 0.35f))
        .parent(simulation->robot())
        .position(0.0f, 0.0f, -0.001f)
        .scale(3.0f);
    for (auto const& object : simulation->scenario().objects)
    {
        detector.add(object.shape.name, object.shape.color);
    }
    view_camera.position(1.4f, 1.1f, 1.4f)
        .add<compages::world::Orbit>(
            compages::core::Vector3f(0.2f, 0.3f, 0.0f));
    if (auto prepared = scene->prepare(); !prepared)
    {
        error = prepared.error();
    }
    playing = true;
}
