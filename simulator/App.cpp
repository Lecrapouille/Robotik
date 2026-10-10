// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "App.hpp"

#include "FlyHost.hpp"
#include "SimulatorDisplay.hpp"

#include "Robotik/Scenario/Scenario.hpp"

#include "Compages/Core/Units.hpp"
#include "Compages/World/Controllers/Controls.hpp"

namespace
{
void tuneViewOrbit(compages::world::World& p_world,
                   compages::world::Entity p_camera)
{
    if (compages::world::Orbit* orbit =
            p_world.behavior<compages::world::Orbit>(p_camera))
    {
        // Default 2.5f × raw scroll is harsh; Orbit::start() only raises this.
        orbit->controller.zoom_sensitivity = 0.35f;
    }
}
} // namespace

#include <fstream>
#include <iostream>
#include <iterator>

#define SIMULATOR_DT_S 0.01
#define SIMULATOR_MAX_STEPS_PER_FRAME 8
#define RL_PAUSE_AFTER_S 1.2

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
        std::cout << "  " << run.start.value() << " s " << skills.name(run.skill)
                  << ' ' << robotik::toString(run.state) << ' '
                  << robotik::toString(run.reason)
                  << (run.blocker != robotik::NO_RESOURCE
                          ? " " + resources.name(run.blocker)
                          : "")
                  << '\n';
    }
    std::size_t passed = 0;
    auto const checks = p_simulation.checks();
    for (auto const& check : checks)
    {
        passed += check.passed ? 1u : 0u;
    }
    std::cout << "Mission "
              << (p_simulation.status() == bt::Status::SUCCESS ? "SUCCESS"
                                                              : "FAILURE")
              << " seed=" << p_simulation.seed().value
              << " time=" << p_simulation.time().value() << " s checks="
              << passed << '/' << checks.size() << std::endl;
}

void App::load()
{
    error.clear();
    scenario_title.clear();
    scenario_text.clear();
    owns_clock = false;
    teach.manual = false;
    teach.pendant.clear();
    teach.markers.clear();
    // The fly robot is parented in the world. Drop it before the world.
    fly = {};
    plugins.stop();
    plugins.detach();
    simulation.reset();
    mission.reset();
    plugins.release();
    scene_view.reset();
    scene.reset();
    world = std::make_unique<compages::world::World>();
    scene = std::make_unique<compages::renderer::Scene>(*world);
    auto const clear = viewClearColor();
    scene->background(clear.x, clear.y, clear.z).ambient(0.35f, 0.35f, 0.38f);
    scene->sun("Sun").rotation(Radians(-0.9f),
                               compages::core::Vector3f(1.0f, 0.3f, 0.0f));
    view_camera = scene->camera("ViewCamera");
    scene->activeCamera(view_camera);
    scene_view = std::make_unique<SimulatorView>(*scene);
    try
    {
        if (std::ifstream file(scenario_path); file)
        {
            scenario_text.assign(std::istreambuf_iterator<char>(file),
                                 std::istreambuf_iterator<char>());
        }
        if (plugins.catalog().packages().empty())
        {
            plugins.scan();
        }
        robotik::PluginMatch const matched = plugins.catalog().matchScenario(scenario_path);
        if (matched.scenario != nullptr)
        {
            scenario_title = matched.scenario->name;
        }
        if (matched.package != nullptr && matched.package->graphics)
        {
            if (matched.package->id != "robotik.fly")
            {
                error = "this graphics demo has no viewer";
                return;
            }
            view_camera.position(4.0f, 5.5f, 7.0f)
                .add<compages::world::Orbit>(
                    compages::core::Vector3f(4.0f, 1.2f, 0.0f));
            tuneViewOrbit(*world, view_camera);
            loadFly(*this);
            m_lag = 0.0;
            m_reported = false;
            playing = true;
            return;
        }
        robotik::Scenario scenario = robotik::Scenario::load(scenario_path);
        robotik::PluginPrepare const prepared = plugins.prepare(scenario_path);
        if (prepared != robotik::PluginPrepare::Ready)
        {
            error = plugins.error().empty() ? "scenario is not a demo package" : plugins.error();
            return;
        }
        owns_clock = plugins.ownsClock();
        scenario_title = plugins.scenarioName();
        simulation = std::make_unique<robotik::Simulation>(
            *world, std::move(scenario), scene_view.get(), plugins.mission());
        plugins.attach(simulation.get());
        if (plugins.start() != ROBOTIK_PLUGIN_OK)
        {
            error = plugins.error().empty() ? "plugin start failed" : plugins.error();
            plugins.detach();
            simulation.reset();
            plugins.release();
            return;
        }
    }
    catch (std::exception const& failure)
    {
        error = failure.what();
        fly = {};
        plugins.detach();
        simulation.reset();
        plugins.release();
        return;
    }
    seed = simulation->seed().value;

    if (!scene_view->hasGround())
    {
        scene->plane("Floor", compages::renderer::color(0.32f, 0.33f, 0.35f))
            .parent(simulation->robot().root())
            .position(0.0f, 0.0f, -0.001f)
            .scale(3.0f);
    }
    // Compages world is Y-up. The robot stands on XZ after the URDF
    // conversion; look at the table from above-front. A package may override it.
    robotik::PluginPackage const* package = plugins.catalog().matchScenario(scenario_path).package;
    if (package != nullptr && package->has_view)
    {
        view_camera.position(package->eye[0], package->eye[1], package->eye[2])
            .add<compages::world::Orbit>(compages::core::Vector3f(
                package->target[0], package->target[1], package->target[2]));
    }
    else
    {
        view_camera.position(1.6f, 1.2f, 1.6f)
            .add<compages::world::Orbit>(
                compages::core::Vector3f(0.35f, 0.15f, 0.0f));
    }
    tuneViewOrbit(*world, view_camera);
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
    if (fly.environment)
    {
        resetFly(*this, p_seed);
        m_lag = 0.0;
        m_reported = false;
        return;
    }
    if (!simulation)
    {
        return;
    }
    seed = p_seed;
    simulation->reset(robotik::Seed{ p_seed });
    m_lag = 0.0;
    m_reported = false;
}

void App::advanceFly(double p_elapsed)
{
    if (!fly.environment)
    {
        return;
    }
    if (step_once)
    {
        step_once = false;
        stepFly(*this);
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
        if (!fly.done)
        {
            stepFly(*this);
        }
        else
        {
            fly.pause += SIMULATOR_DT_S;
            if (fly.pause >= RL_PAUSE_AFTER_S)
            {
                reset(seed + 1u);
            }
            m_lag = 0.0;
            break;
        }
        m_lag -= SIMULATOR_DT_S;
        ++steps;
    }
    if (steps == SIMULATOR_MAX_STEPS_PER_FRAME)
    {
        m_lag = 0.0;
    }
}

void App::advance(double p_elapsed)
{
    if (fly.environment)
    {
        advanceFly(p_elapsed);
        return;
    }
    if (!simulation)
    {
        return;
    }
    if (owns_clock)
    {
        if (step_once)
        {
            step_once = false;
            plugins.afterStep(SIMULATOR_DT_S);
        }
        else if (!playing)
        {
            plugins.setPaused(true);
            m_lag = 0.0;
        }
        else
        {
            plugins.setPaused(false);
            plugins.afterStep(p_elapsed * static_cast<double>(speed));
        }
        seed = simulation->seed().value;
        return;
    }
    if (simulation->finished() && !m_reported)
    {
        m_reported = true;
        report(*simulation);
    }
    if (step_once)
    {
        step_once = false;
        if (teach.manual)
        {
            teach.pendant.update(simulation->robot(), Seconds(SIMULATOR_DT_S));
        }
        simulation->step(Seconds(SIMULATOR_DT_S));
        plugins.afterStep(SIMULATOR_DT_S);
        return;
    }
    if (!playing)
    {
        plugins.setPaused(true);
        m_lag = 0.0;
        return;
    }
    plugins.setPaused(false);
    m_lag += p_elapsed * static_cast<double>(speed);
    int steps = 0;
    while (m_lag >= SIMULATOR_DT_S && steps < SIMULATOR_MAX_STEPS_PER_FRAME)
    {
        if (teach.manual)
        {
            teach.pendant.update(simulation->robot(), Seconds(SIMULATOR_DT_S));
        }
        simulation->step(Seconds(SIMULATOR_DT_S));
        plugins.afterStep(SIMULATOR_DT_S);
        m_lag -= SIMULATOR_DT_S;
        ++steps;
    }
    if (steps == SIMULATOR_MAX_STEPS_PER_FRAME)
    {
        m_lag = 0.0;
    }
}
