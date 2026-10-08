// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "FlyHost.hpp"

#include "App.hpp"
#include "FlyDraw.hpp"
#include "SimulatorDisplay.hpp"

#include "Robotik/Robot/Robot.hpp"

#include "Compages/GPU/RenderPass.hpp"

#include <algorithm>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>

namespace
{

constexpr std::size_t TRAIL = 240;

//! @brief URDF, obstacle jitter, the three beams and the trail crumbs.
//! Unused crumbs sit under the floor.
void poseFly(App& p_app)
{
    FlySnapshot const& snapshot = p_app.fly.environment->snapshot();
    poseFlyBody(*p_app.fly.robot, snapshot);
    p_app.fly.robot->step(Seconds(p_app.fly.environment->dt()));

    std::size_t const boxes = p_app.fly.obstacles.size();
    std::span<FlyBox const> const obstacles =
        p_app.fly.environment->obstacles();
    for (std::size_t i = 0; i < boxes && i < obstacles.size(); ++i)
    {
        compages::core::Vector3f const at = flyToView(obstacles[i].position);
        p_app.fly.obstacles[i].position(at.x, at.y, at.z);
    }

    for (int ray = 0; ray < 3; ++ray)
    {
        float const value = ray == 0 ? snapshot.vision.left
                                     : (ray == 1 ? snapshot.vision.center
                                                 : snapshot.vision.right);
        aimFlyBeam(p_app.fly.beams[ray],
                   p_app.fly.beam_from[ray],
                   p_app.fly.beam_to[ray],
                   snapshot.vision.origins[ray],
                   snapshot.vision.ends[ray],
                   value);
    }

    std::span<robotik::Vector3 const> const trail =
        p_app.fly.environment->trail();
    std::size_t const shown = std::min(trail.size(), p_app.fly.trail.size());
    for (std::size_t i = 0; i < shown; ++i)
    {
        compages::core::Vector3f const at = flyToView(trail[i]);
        p_app.fly.trail[i].position(at.x, at.y, at.z).scale(0.04f);
    }
    for (std::size_t i = shown; i < p_app.fly.trail.size(); ++i)
    {
        p_app.fly.trail[i].position(0.0f, -20.0f, 0.0f);
    }
}

} // namespace

void loadFly(App& p_app)
{
    // Load the scenario.
    FlyScenario scenario = FlyScenario::load(p_app.scenario_path);

    // Create the robot session.
    p_app.fly.robot = std::make_unique<robotik::RobotSession>(
        *p_app.world, scenario.robot_model, p_app.scene_view.get());
    p_app.fly.eyes[0] =
        mountFlyEye(*p_app.scene, *p_app.fly.robot, "eye_L", "EyeLeft");
    p_app.fly.eyes[1] =
        mountFlyEye(*p_app.scene, *p_app.fly.robot, "eye_R", "EyeRight");

    // Same props as the standalone window: brown boxes, gold food, green floor.
    // Box height is Z-up, so it becomes the Compages Y scale.
    auto brown = compages::renderer::color(0.45f, 0.32f, 0.18f);
    int index = 0;

    // Create the obstacles.
    for (FlyBox const& box : scenario.obstacles)
    {
        compages::core::Vector3f const at = flyToView(box.position);
        p_app.fly.obstacles.push_back(
            p_app.scene->box("obstacle_" + std::to_string(index), brown)
                .position(at.x, at.y, at.z)
                .scale(static_cast<float>(box.size.x),
                       static_cast<float>(box.size.z),
                       static_cast<float>(box.size.y)));
        ++index;
    }

    // Create the food.
    compages::core::Vector3f const food = flyToView(scenario.target);
    p_app.scene->sphere("food", compages::renderer::color(0.95f, 0.75f, 0.15f))
        .position(food.x, food.y, food.z)
        .scale(0.18f);
    compages::core::Vector3f const middle =
        flyToView(robotik::Vector3(scenario.arena.x * 0.5, 0.0, 0.0));

    // Create the floor.
    p_app.scene->plane("Floor", compages::renderer::color(0.55f, 0.62f, 0.38f))
        .position(middle.x, 0.0f, middle.z)
        .rotation(Radians(-0.5f * 3.14159265358979323846f),
                  compages::core::Vector3f(1.0f, 0.0f, 0.0f))
        .scale(static_cast<float>(scenario.arena.x + 4.0),
               static_cast<float>(scenario.arena.y + 4.0),
               1.0f);

    // Create the beams.
    auto const left = compages::renderer::color(0.95f, 0.75f, 0.2f);
    auto const center = compages::renderer::color(0.95f, 0.25f, 0.2f);
    auto const right = compages::renderer::color(0.25f, 0.45f, 1.0f);
    compages::renderer::Look const looks[3] = { left, center, right };
    char const* names[3] = { "ray_left", "ray_center", "ray_right" };
    for (int ray = 0; ray < 3; ++ray)
    {
        // Create the beam.
        p_app.fly.beams[ray] =
            p_app.scene->cone(std::string(names[ray]), looks[ray]);
        p_app.fly.beam_from[ray] =
            p_app.scene->sphere(std::string(names[ray]) + "_from", looks[ray]);
        p_app.fly.beam_to[ray] =
            p_app.scene->sphere(std::string(names[ray]) + "_to", looks[ray]);
    }

    // Create the trail crumbs.
    auto crumb = compages::renderer::color(0.95f, 0.95f, 0.9f);
    p_app.fly.trail.reserve(TRAIL);
    for (std::size_t i = 0; i < TRAIL; ++i)
    {
        p_app.fly.trail.push_back(
            p_app.scene->sphere("trail_" + std::to_string(i), crumb)
                .position(0.0f, -20.0f, 0.0f)
                .scale(0.04f));
    }

    // Prepare the scene.
    if (auto prepared = p_app.scene->prepare(); !prepared)
    {
        throw std::runtime_error(prepared.error());
    }

    // Create the environment and the brain.
    p_app.fly.environment =
        std::make_unique<FlyEnvironment>(std::move(scenario), 0);
    p_app.fly.brain = std::make_unique<FlyBrain>(
        FlyBrain::Kind::Shiu,
        static_cast<float>(p_app.fly.environment->scenario().target.z));

    // Reset the fly.
    p_app.seed = p_app.fly.environment->scenario().seed;
    resetFly(p_app, p_app.seed);
}

void resetFly(App& p_app, std::uint64_t p_seed)
{
    if (!p_app.fly.environment || !p_app.fly.brain)
        return;

    p_app.seed = p_seed;
    p_app.fly.brain->reset(robotik::Seed{ p_seed });
    p_app.fly.environment->reset(robotik::Seed{ p_seed },
                                 p_app.fly.observation);
    p_app.fly.action.fill(0.0f);
    p_app.fly.episode_return = 0.0f;
    p_app.fly.done = false;
    p_app.fly.success = false;
    p_app.fly.pause = 0.0;
    poseFly(p_app);
}

//! @brief One decision and one environment step. A finished episode waits;
//! the toolbar pause timer lives in the caller.
void stepFly(App& p_app)
{
    if (!p_app.fly.environment || p_app.fly.done)
        return;

    // Calculate the observation and action.
    FlyObservation const observation =
        FlyObservation::from(p_app.fly.observation);

    // Update the brain.
    FlyAction const action = p_app.fly.brain->update(
        observation, Seconds(p_app.fly.environment->dt()));
    action.write(p_app.fly.action);

    // Step the environment.
    robotik::StepResult const result =
        p_app.fly.environment->step(p_app.fly.action, p_app.fly.observation);
    p_app.fly.episode_return += result.reward;

    // Check if the episode is finished.
    if (result.done())
    {
        p_app.fly.done = true;
        p_app.fly.success = p_app.fly.environment->snapshot().reached;
        p_app.fly.pause = 0.0;
    }
    poseFly(p_app);
}

void renderFlyEyes(App& p_app)
{
    if (!p_app.scene)
    {
        return;
    }
    for (int eye = 0; eye < 2; ++eye)
    {
        if (!p_app.fly.eyes[eye])
        {
            continue;
        }
        RenderTarget& picture = p_app.fly.eye_picture[eye];
        if (!picture.resize(320, 240))
        {
            continue;
        }
        compages::gpu::RenderPass pass(
            picture.framebuffer,
            compages::gpu::PassDesc{ .color = viewClearColor() });
        p_app.scene->render(p_app.fly.eyes[eye]);
    }
}

void selectFlyBrain(App& p_app, bool p_connectome)
{
    if (!p_app.fly.environment)
    {
        return;
    }

    // Keep the flying brain until the new one is fully loaded.
    float const cruise = static_cast<float>(
        p_app.fly.environment->scenario().target.z);
    try
    {
        auto brain = std::make_unique<FlyBrain>(FlyBrain::Kind::Shiu, cruise);
        if (p_connectome)
        {
            std::cerr << "Loading FlyWire connectome from " << FLYWIRE_EDGES
                      << "\n";
            brain->loadConnectome(FLYWIRE_EDGES, FLYWIRE_BINDING);
        }
        p_app.fly.brain = std::move(brain);
        p_app.fly.connectome = p_connectome;
        p_app.error.clear();
        resetFly(p_app, p_app.seed);
    }
    catch (std::exception const& error)
    {
        p_app.error = std::string(error.what()) +
                      ". Run make download-external-libs && make "
                      "compile-external-libs";
    }
}
