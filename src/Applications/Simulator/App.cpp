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

#define SIMULATOR_DT_S 0.01
#define SIMULATOR_MAX_STEPS_PER_FRAME 8

#include <iostream>

// One line per mission end, so runs can be compared with Robotik-Headless.
static void report(robotik::Simulation const& p_simulation)
{
    robotik::SkillScheduler const& skills = p_simulation.skills();
    robotik::ResourceManager const& resources = p_simulation.robot().resources();
    for (robotik::SkillRun const& run : skills.trace())
    {
        if (run.state == robotik::SkillState::Succeeded)
        {
            continue;
        }
        std::cout << "  " << run.start.value() << " s " << skills.name(run.skill) << ' '
                  << robotik::toString(run.state) << ' ' << robotik::toString(run.reason)
                  << (run.blocker != robotik::NO_RESOURCE ? " " + resources.name(run.blocker) : "")
                  << '\n';
    }
    for (robotik::WorldObject const& object : p_simulation.worldModel().objects())
    {
        std::cout << "  belief " << object.name << ' ' << object.position.x << ' '
                  << object.position.y << ' ' << object.position.z << " seen "
                  << object.seen.value() << '\n';
    }
    std::size_t passed = 0;
    auto const checks = p_simulation.checks();
    for (auto const& check : checks)
    {
        passed += check.passed ? 1u : 0u;
    }
    std::cout << "Mission " << (p_simulation.status() == bt::Status::SUCCESS ? "SUCCESS" : "FAILURE")
              << " seed=" << p_simulation.seed().value
              << " time=" << p_simulation.time().value() << " s checks="
              << passed << '/' << checks.size() << std::endl;
}

void App::load()
{
    error.clear();
    detector = nullptr;
    simulation.reset();
    scene_view.reset();
    scene.reset();
    world = std::make_unique<compages::world::World>();
    scene = std::make_unique<compages::renderer::Scene>(*world);
    auto const clear = viewClearColor();
    scene->background(clear.x, clear.y, clear.z).ambient(0.35f, 0.35f, 0.38f);
    scene->sun("Sun")
        .rotation(Radians(-0.9f), compages::core::Vector3f(1.0f, 0.3f, 0.0f));
    view_camera = scene->camera("ViewCamera");
    scene->activeCamera(view_camera);
    scene_view = std::make_unique<SimulatorView>(*scene);
    try
    {
        simulation = std::make_unique<robotik::Simulation>(
            *world, robotik::Scenario::load(scenario_path), scene_view.get());
    }
    catch (std::exception const& failure)
    {
        error = failure.what();
        simulation.reset();
        return;
    }
    seed = simulation->seed().value;

    scene->plane("Floor", compages::renderer::color(0.32f, 0.33f, 0.35f))
        .parent(simulation->robot().root())
        .position(0.0f, 0.0f, -0.001f)
        .scale(3.0f);
    detector = &simulation->perception().add<ColorDetector>();
    for (auto const& object : simulation->scenario().objects)
    {
        detector->add(object.shape.name, object.shape.color);
    }
    view_camera.position(1.4f, 1.1f, 1.4f)
        .add<compages::world::Orbit>(
            compages::core::Vector3f(0.2f, 0.3f, 0.0f));
    if (auto prepared = scene->prepare(); !prepared)
    {
        error = prepared.error();
    }
    m_lag = 0.0;
    m_reported = false;
    playing = true;
}

void App::reset(std::uint64_t p_seed)
{
    if (!simulation)
    {
        return;
    }
    seed = p_seed;
    simulation->reset(robotik::Seed{ p_seed });
    m_lag = 0.0;
    m_reported = false;
}

void App::advance(double p_elapsed)
{
    if (!simulation)
    {
        return;
    }
    if (simulation->finished() && !m_reported)
    {
        m_reported = true;
        report(*simulation);
    }
    Seconds const dt(SIMULATOR_DT_S);
    if (step_once)
    {
        step_once = false;
        simulation->step(dt);
        return;
    }
    if (!playing)
    {
        m_lag = 0.0;
        return;
    }
    m_lag += p_elapsed * static_cast<double>(speed);
    int steps = 0;
    while (m_lag >= SIMULATOR_DT_S && steps < SIMULATOR_MAX_STEPS_PER_FRAME)
    {
        simulation->step(dt);
        m_lag -= SIMULATOR_DT_S;
        ++steps;
    }
    if (steps == SIMULATOR_MAX_STEPS_PER_FRAME)
    {
        m_lag = 0.0;
    }
}
