// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "App.hpp"

#include "SimulatorDisplay.hpp"

#include "Robotik/Scenario/Scenario.hpp"

#include "Robotik/Math/Si.hpp"
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

static std::string readText(std::filesystem::path const& p_path)
{
    std::ifstream file(p_path);
    if (!file)
    {
        return {};
    }
    return std::string(std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>());
}

void JointHistory::clear()
{
    time.clear();
    position.clear();
    velocity.clear();
    acceleration.clear();
    effort.clear();
    head = 0;
    count = 0;
}

std::size_t JointHistory::latest() const
{
    if (count == 0)
    {
        return CAPACITY;
    }
    return (head + count - 1u) % CAPACITY;
}

void JointHistory::record(robotik::Simulation const& p_simulation)
{
    robotik::JointSet const& joints = p_simulation.robot().joints();
    std::size_t const size = joints.size();
    if (position.size() != size || time.size() != CAPACITY)
    {
        clear();
        time.assign(CAPACITY, 0.0f);
        position.assign(size, std::vector<float>(CAPACITY, 0.0f));
        velocity.assign(size, std::vector<float>(CAPACITY, 0.0f));
        acceleration.assign(size, std::vector<float>(CAPACITY, 0.0f));
        effort.assign(size, std::vector<float>(CAPACITY, 0.0f));
    }
    std::size_t const previous = latest();
    float const now = static_cast<float>(p_simulation.time().value());
    if (previous != CAPACITY && now <= time[previous])
    {
        return;
    }
    std::size_t slot = 0;
    if (count < CAPACITY)
    {
        slot = (head + count) % CAPACITY;
        ++count;
    }
    else
    {
        slot = head;
        head = (head + 1u) % CAPACITY;
    }
    time[slot] = now;
    for (robotik::JointId id = 0; id < size; ++id)
    {
        float const speed = static_cast<float>(joints.velocity(id));
        position[id][slot] = static_cast<float>(joints.position(id));
        velocity[id][slot] = speed;
        effort[id][slot] = static_cast<float>(joints.effort(id));
        float accel = 0.0f;
        if (previous != CAPACITY && now > time[previous])
        {
            accel = (speed - velocity[id][previous]) / (now - time[previous]);
        }
        acceleration[id][slot] = accel;
    }
}

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
    auto const& checks = p_simulation.checks();
    for (robotik::Check const& check : checks)
    {
        passed += check.verdict == robotik::Check::Verdict::Passed ? 1u : 0u;
    }
    std::cout << "Run " << robotik::toString(p_simulation.outcome())
              << " seed=" << p_simulation.seed().value
              << " time=" << p_simulation.time().value() << " s checks="
              << passed << '/' << checks.size() << std::endl;
}

void App::load(bool p_keep_tool)
{
    error.clear();
    scenario_title.clear();
    scenario_text.clear();
    world_path.clear();
    world_text.clear();
    tree_text.clear();
    history.clear();
    ui.realtime_factor = 0.0f;
    owns_clock = false;
    teach.manual = false;
    teach.pendant.clear();
    teach.markers.clear();
    if (!p_keep_tool)
    {
        tool_override = false;
        tool_override_name.clear();
    }
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
        scenario_text = readText(scenario_path);
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
            // The package draws with an API this host does not offer.
            error = "this graphics demo has no viewer";
            return;
        }
        robotik::Scenario scenario =
            robotik::Scenario::load(scenario_path, plugins.assets());
        world_path = robotik::ScenarioSource::load(scenario_path).world_file;
        world_text = readText(world_path);
        tree_text = readText(scenario.controller.behavior_tree);
        if (tool_override)
        {
            scenario.world.mount(tool_override_name);
        }
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
        plugins.detach();
        simulation.reset();
        plugins.release();
        return;
    }
    seed = simulation->seed().value;
    ui.plotted.assign(simulation->robot().joints().size(),
                      simulation->robot().joints().size() <= 8u);
    ui.place_plugin_panels = true;
    ui.focus_plugin_panel = true;
    history.record(*simulation);

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
    playing = false;
}

void App::reset(std::uint64_t p_seed)
{
    if (!simulation)
    {
        return;
    }
    seed = p_seed;
    simulation->reset(robotik::Seed{ p_seed });
    history.clear();
    history.record(*simulation);
    m_lag = 0.0;
    m_reported = false;
}

void App::advance(double p_elapsed)
{
    if (!simulation)
    {
        return;
    }
    double const before = simulation->time().value();
    step(p_elapsed);
    if (simulation)
    {
        history.record(*simulation);
        if (playing && p_elapsed > 0.0)
        {
            float const rate =
                static_cast<float>((simulation->time().value() - before) / p_elapsed);
            ui.realtime_factor = 0.9f * ui.realtime_factor + 0.1f * rate;
        }
        else
        {
            ui.realtime_factor = 0.0f;
        }
    }
}

void App::step(double p_elapsed)
{
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
        history.record(*simulation);
        m_lag -= SIMULATOR_DT_S;
        ++steps;
    }
    if (steps == SIMULATOR_MAX_STEPS_PER_FRAME)
    {
        m_lag = 0.0;
    }
}
