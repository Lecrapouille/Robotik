// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "App.hpp"

#include "SimulatorDisplay.hpp"

#include "Robotik/Package/Scenario/Scenario.hpp"

#include "Compages/World/Controllers/Controls.hpp"
#include "Robotik/Common/Math.hpp"

/// Limits of the camera rig for a robot cell: from a few centimetres to
/// a few metres.
static void tuneViewRig(compages::world::World& p_world,
                        compages::world::Entity p_camera)
{
    if (compages::world::Rig* rig =
            p_world.behavior<compages::world::Rig>(p_camera))
    {
        rig->controller.min_distance = 0.15F;
        rig->controller.max_distance = 20.0F;
    }
}

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
    return std::string(std::istreambuf_iterator<char>(file),
                       std::istreambuf_iterator<char>());
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
    return (head + count - 1U) % CAPACITY;
}

void JointHistory::record(robotik::Simulation const& p_simulation)
{
    robotik::JointSet const& joints = p_simulation.cell().robot().joints();
    std::size_t const size = joints.size();
    if (position.size() != size || time.size() != CAPACITY)
    {
        clear();
        time.assign(CAPACITY, 0.0F);
        position.assign(size, std::vector<float>(CAPACITY, 0.0F));
        velocity.assign(size, std::vector<float>(CAPACITY, 0.0F));
        acceleration.assign(size, std::vector<float>(CAPACITY, 0.0F));
        effort.assign(size, std::vector<float>(CAPACITY, 0.0F));
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
        head = (head + 1U) % CAPACITY;
    }
    time[slot] = now;
    for (robotik::JointId id = 0; id < size; ++id)
    {
        float const speed = static_cast<float>(joints.velocity(id));
        position[id][slot] = static_cast<float>(joints.position(id));
        velocity[id][slot] = speed;
        effort[id][slot] = static_cast<float>(joints.effort(id));
        float accel = 0.0F;
        if (previous != CAPACITY && now > time[previous])
        {
            accel = (speed - velocity[id][previous]) / (now - time[previous]);
        }
        acceleration[id][slot] = accel;
    }
}

static void report(robotik::Simulation const& p_simulation)
{
    robotik::SkillScheduler const& skills = p_simulation.cell().skills();
    robotik::ResourceManager const& resources =
        p_simulation.cell().robot().resources();
    for (robotik::SkillRun const& run : skills.trace())
    {
        if (run.state == robotik::SkillState::Succeeded)
        {
            continue;
        }
        std::cout << "  " << run.start.value() << " s "
                  << skills.name(run.skill) << ' '
                  << robotik::toString(run.state) << ' '
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
        passed += check.verdict == robotik::Check::Verdict::Passed ? 1U : 0U;
    }
    std::cout << "Run " << robotik::toString(p_simulation.outcome())
              << " seed=" << p_simulation.seed().value
              << " time=" << p_simulation.time().value()
              << " s checks=" << passed << '/' << checks.size() << std::endl;
}

App::~App()
{
    // The members go in reverse order: the session, declared first, would
    // disconnect from the signals of a simulation already freed.
    plugins.stop();
    plugins.detach();
    simulation.reset();
    mission.reset();
    plugins.release();
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
    ui.realtime_factor = 0.0F;
    ui.manual_faults.clear();
    ui.fault_pick.clear();
    ui.chip_hold.clear();
    owns_clock = false;
    teach.manual = false;
    teach.pendant.clear();
    teach.pendant.frame({});
    teach.markers.clear();
    teach.axes_made = false;
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
    scene->background(clear.x, clear.y, clear.z).ambient(0.35F, 0.35F, 0.38F);
    // A sun about 40 degrees high, from the side: the shadows stretch on
    // the floor and show where each object stands.
    scene->sun("Sun").rotation(robotik::Radians(-0.75F),
                               compages::core::Vector3f(1.0F, 0.0F, 0.6F));
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
        robotik::PluginMatch const matched =
            plugins.catalog().matchScenario(scenario_path);
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
        robotik::Result<robotik::Scenario> loaded =
            robotik::loadScenario(scenario_path, plugins.assets());
        if (!loaded)
        {
            error = loaded.error();
            return;
        }
        robotik::Scenario scenario = loaded.take();
        world_path = robotik::loadScenarioSource(scenario_path).value().world_file;
        world_text = readText(world_path);
        tree_text = readText(scenario.controller.behavior_tree);
        if (tool_override)
        {
            if (robotik::Result<void> mounted = robotik::mountTool(scenario.world, tool_override_name);
                !mounted)
            {
                error = mounted.error();
                return;
            }
        }
        robotik::PluginPrepare const prepared = plugins.prepare(scenario_path);
        if (prepared != robotik::PluginPrepare::Ready)
        {
            error = plugins.error().empty() ? "scenario is not a demo package"
                                            : plugins.error();
            return;
        }
        owns_clock = plugins.ownsClock();
        scenario_title = plugins.scenarioName();
        simulation = std::make_unique<robotik::Simulation>(
            *world, std::move(scenario), scene_view.get(), plugins.mission());
        plugins.attach(simulation.get());
        if (plugins.start() != ROBOTIK_PLUGIN_OK)
        {
            error = plugins.error().empty() ? "plugin start failed"
                                            : plugins.error();
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
    ui.plotted.assign(simulation->cell().robot().joints().size(),
                      simulation->cell().robot().joints().size() <= 8U);
    ui.place_plugin_panels = true;
    ui.focus_plugin_panel = true;
    history.record(*simulation);

    if (!scene_view->hasGround())
    {
        // A grid on the floor: distances and heights read at a glance.
        static std::filesystem::path const grid = floorGrid();
        compages::world::Entity floor =
            scene->plane("Floor", compages::renderer::texture(grid.string()));
        // In the world, not on the robot: a floating base would carry it.
        floor.parent(simulation->cell().worldFrame())
            .position(0.0F, 0.0F, -0.001F)
            .scale(3.0F);
        noShadow(floor);
    }
    // Compages world is Y-up. The robot stands on XZ after the URDF
    // conversion; look at the table from above-front. A package may override
    // it.
    robotik::PluginPackage const* package =
        plugins.catalog().matchScenario(scenario_path).package;
    if (package != nullptr && package->has_view)
    {
        view_camera.position(package->eye[0], package->eye[1], package->eye[2])
            .add<compages::world::Rig>(compages::core::Vector3f(
                package->target[0], package->target[1], package->target[2]));
    }
    else
    {
        view_camera.position(1.6F, 1.2F, 1.6F)
            .add<compages::world::Rig>(
                compages::core::Vector3f(0.35F, 0.15F, 0.0F));
    }
    tuneViewRig(*world, view_camera);
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
    ui.manual_faults.clear();
    simulation->reset(robotik::Seed{ p_seed });
    // Replay (or a new seed) means: run it again, even from a pause.
    playing = true;
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
            float const rate = static_cast<float>(
                (simulation->time().value() - before) / p_elapsed);
            ui.realtime_factor = 0.9F * ui.realtime_factor + 0.1F * rate;
        }
        else
        {
            ui.realtime_factor = 0.0F;
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
        // The run is over: freeze the scene on its last state. Manual mode
        // (Joints, Teach Pendant) still moves the robot.
        playing = false;
    }
    if (step_once)
    {
        step_once = false;
        if (teach.manual)
        {
            teach.pendant.update(simulation->cell().robot(), robotik::Seconds(SIMULATOR_DT_S));
        }
        simulation->step(robotik::Seconds(SIMULATOR_DT_S));
        plugins.afterStep(SIMULATOR_DT_S);
        return;
    }
    // In Manual mode the mission is suspended and the robot is live: the
    // operator moves it without pressing Play.
    if (!playing && !teach.manual)
    {
        plugins.setPaused(true);
        m_lag = 0.0;
        return;
    }
    plugins.setPaused(false);
    m_lag += p_elapsed * static_cast<double>(playing ? speed : 1.0F);
    int steps = 0;
    while (m_lag >= SIMULATOR_DT_S && steps < SIMULATOR_MAX_STEPS_PER_FRAME)
    {
        if (teach.manual)
        {
            teach.pendant.update(simulation->cell().robot(), robotik::Seconds(SIMULATOR_DT_S));
        }
        simulation->step(robotik::Seconds(SIMULATOR_DT_S));
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
