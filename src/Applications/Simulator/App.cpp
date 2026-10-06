// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "App.hpp"

#include "SimulatorDisplay.hpp"

#include "Robotik/Robot/Actuators.hpp"
#include "Robotik/Scenario/Scenario.hpp"

#include "Compages/Core/Units.hpp"
#include "Compages/World/Controllers/Controls.hpp"

#include <algorithm>
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

void App::select(HostedMission p_kind)
{
    kind = p_kind;
    switch (kind)
    {
        case HostedMission::LineFollower:
            scenario_path = "data/scenarios/line_follower.yml";
            break;
        case HostedMission::PickPlace:
        case HostedMission::PickPlaceRl:
            scenario_path = "data/scenarios/pick_and_place.yml";
            break;
    }
    load();
}

void App::load()
{
    error.clear();
    scenario_text.clear();
    detector = nullptr;
    line_follower = nullptr;
    RlWatch const keep = rl;
    rl = {};
    rl.converged = keep.converged;
    rl.mix = keep.mix;
    rl.auto_repeat = keep.auto_repeat;
    rl.spread = keep.spread;
    rl.max_steps = keep.max_steps;
    rl.delivered = keep.delivered;
    rl.failed = keep.failed;
    simulation.reset();
    mission.reset();
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
        robotik::Scenario scenario = robotik::Scenario::load(scenario_path);
        if (kind == HostedMission::PickPlaceRl)
        {
            scenario.behavior_tree.clear();
            scenario.faults.clear();
            scenario.random_faults.clear();
            for (auto& object : scenario.objects)
            {
                if (object.shape.name == PICK_PLACE_CUBE)
                {
                    object.randomize[0] = { -rl.spread, rl.spread };
                    object.randomize[1] = { -rl.spread, rl.spread };
                }
            }
            auto pick = std::make_unique<PickPlaceMission>(false);
            detector = pick->detector();
            mission = std::move(pick);
        }
        else if (kind == HostedMission::LineFollower)
        {
            auto line = std::make_unique<LineFollowerMission>(1.0);
            line_follower = line.get();
            mission = std::move(line);
        }
        else
        {
            auto pick = std::make_unique<PickPlaceMission>(true);
            mission = std::move(pick);
        }
        simulation = std::make_unique<robotik::Simulation>(
            *world, std::move(scenario), scene_view.get(), mission.get());
        if (auto* pick = dynamic_cast<PickPlaceMission*>(mission.get()))
        {
            detector = pick->detector();
        }
    }
    catch (std::exception const& failure)
    {
        error = failure.what();
        simulation.reset();
        return;
    }
    seed = simulation->seed().value;
    rl.noise = robotik::Random(robotik::Seed{ seed }.derive("policy"));

    if (!scene_view->hasGround())
    {
        scene->plane("Floor", compages::renderer::color(0.32f, 0.33f, 0.35f))
            .parent(simulation->robot().root())
            .position(0.0f, 0.0f, -0.001f)
            .scale(3.0f);
    }
    // Compages world is Y-up. The robot stands on XZ after the URDF
    // conversion; look at the table from above-front.
    if (kind == HostedMission::LineFollower)
    {
        view_camera.position(0.0f, 7.0f, 6.0f)
            .add<compages::world::Orbit>(compages::core::Vector3f(0.0f, 0.0f, 0.0f));
    }
    else
    {
        view_camera.position(1.6f, 1.2f, 1.6f)
            .add<compages::world::Orbit>(
                compages::core::Vector3f(0.35f, 0.15f, 0.0f));
    }
    if (kind == HostedMission::PickPlaceRl)
    {
        rl.arm = simulation->robot().actuators().find<robotik::JointGroup>("arm");
        rl.gripper =
            simulation->robot().actuators().first<robotik::VacuumGripper>();
        if (rl.arm == nullptr || rl.gripper == nullptr)
        {
            error = "RL needs an arm joint group and a vacuum gripper";
            simulation.reset();
            return;
        }
        readyPosture(simulation->robot(), *rl.gripper);
        simulation->observe();
        resetRl();
    }
    if (auto prepared = scene->prepare(); !prepared)
    {
        error = prepared.error();
    }
    m_lag = 0.0;
    m_reported = false;
    playing = true;
}

void App::resetRl()
{
    if (!simulation || rl.gripper == nullptr)
    {
        return;
    }
    writePickPlaceObservation(
        *simulation,
        *rl.gripper,
        0,
        static_cast<std::uint32_t>(rl.max_steps),
        rl.observation);
    rl.steps = 0;
    rl.episode_return = 0.0f;
    rl.grasped = false;
    rl.done = false;
    rl.success = false;
    m_rl_pause = 0.0;
}

void App::reset(std::uint64_t p_seed)
{
    if (!simulation)
    {
        return;
    }
    seed = p_seed;
    simulation->reset(robotik::Seed{ p_seed });
    if (kind == HostedMission::PickPlaceRl && rl.gripper != nullptr)
    {
        readyPosture(simulation->robot(), *rl.gripper);
        simulation->observe();
        resetRl();
    }
    m_lag = 0.0;
    m_reported = false;
}

void App::stepRl()
{
    if (rl.done || rl.arm == nullptr || rl.gripper == nullptr)
    {
        return;
    }
    if (rl.converged)
    {
        pickPlacePolicy(rl.observation, rl.action, 1.0f, nullptr);
    }
    else
    {
        pickPlacePolicy(rl.observation, rl.action, rl.mix, &rl.noise);
        rl.mix = std::min(1.0f, rl.mix + PICK_PLACE_TRAIN_STEP);
        if (rl.mix >= 1.0f)
        {
            rl.converged = true;
        }
    }
    applyPickPlaceAction(*simulation, *rl.arm, *rl.gripper, rl.action);
    simulation->observe();
    ++rl.steps;
    writePickPlaceObservation(
        *simulation,
        *rl.gripper,
        rl.steps,
        static_cast<std::uint32_t>(rl.max_steps),
        rl.observation);

    float reward = 0.0f;
    robotik::WorldModel const& beliefs = simulation->worldModel();
    if (auto const* cube = beliefs.find(PICK_PLACE_CUBE))
    {
        robotik::Vector3 const tip = rl.gripper->tip(simulation->robot());
        if (rl.gripper->holding())
        {
            auto const* box = beliefs.find(PICK_PLACE_BOX);
            robotik::Vector3 const above =
                box != nullptr
                    ? robotik::Vector3{ box->position.x, box->position.y,
                                        box->position.z + 0.1 }
                    : tip;
            reward = 1.0f - static_cast<float>(
                robotik::norm(cube->position - above));
            if (!rl.grasped)
            {
                rl.grasped = true;
                reward += 2.0f;
            }
        }
        else
        {
            robotik::Vector3 const top{ cube->position.x, cube->position.y,
                                        cube->position.z + 0.02 };
            reward = -static_cast<float>(robotik::norm(tip - top));
        }
    }
    if (cubeInBox(*simulation, *rl.gripper))
    {
        reward += 10.0f;
        rl.done = true;
        rl.success = true;
    }
    rl.episode_return += reward;
    if (!rl.done && rl.steps >= static_cast<std::uint32_t>(rl.max_steps))
    {
        rl.done = true;
    }
    if (rl.done)
    {
        if (rl.success)
        {
            ++rl.delivered;
        }
        else
        {
            ++rl.failed;
        }
        m_rl_pause = 0.0;
    }
}

void App::advance(double p_elapsed)
{
    if (!simulation)
    {
        return;
    }
    if (kind != HostedMission::PickPlaceRl && simulation->finished() &&
        !m_reported)
    {
        m_reported = true;
        report(*simulation);
    }
    if (step_once)
    {
        step_once = false;
        if (kind == HostedMission::PickPlaceRl)
        {
            stepRl();
        }
        else
        {
            simulation->step(Seconds(SIMULATOR_DT_S));
        }
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
        if (kind == HostedMission::PickPlaceRl)
        {
            if (!rl.done)
            {
                stepRl();
            }
            else if (rl.auto_repeat)
            {
                m_rl_pause += SIMULATOR_DT_S;
                if (m_rl_pause >= RL_PAUSE_AFTER_S)
                {
                    reset(seed + 1u);
                }
            }
            m_lag = 0.0;
            break;
        }
        simulation->step(Seconds(SIMULATOR_DT_S));
        m_lag -= SIMULATOR_DT_S;
        ++steps;
    }
    if (steps == SIMULATOR_MAX_STEPS_PER_FRAME)
    {
        m_lag = 0.0;
    }
}
