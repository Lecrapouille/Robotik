// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "Robotik/Runtime/Simulation.hpp"

#include "Robotik/Backends/MujocoBackend.hpp"
#include "Robotik/ECS/Queries.hpp"
#include "Robotik/Runtime/RobotRuntime.hpp"
#include "Robotik/Skills/HomeSkill.hpp"
#include "Robotik/Skills/PickPlaceSkills.hpp"
#include "Robotik/Systems/GraspSystem.hpp"

#include "Compages/Renderer/Scene.hpp"
#include "Compages/World/Components/Camera.hpp"
#include "Compages/World/World.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <regex>
#include <stdexcept>

namespace robotik
{

namespace
{

constexpr float kWall = 0.005f;
constexpr double kApproachClearance = 0.10;

// A container: floor and four walls, all children of p_parent.
void walls(compages::renderer::Scene& p_scene,
           compages::world::Entity p_parent,
           ecs::SceneObject const& p_shape)
{
    auto const look =
        compages::renderer::color(p_shape.color[0], p_shape.color[1], p_shape.color[2]);
    float const x = p_shape.size[0];
    float const y = p_shape.size[1];
    float const z = p_shape.size[2];
    auto part = [&](char const* p_name, float p_px, float p_py, float p_pz, float p_sx,
                    float p_sy, float p_sz)
    {
        p_scene.box(p_shape.name + p_name, look)
            .parent(p_parent)
            .position(p_px, p_py, p_pz)
            .scale(p_sx, p_sy, p_sz);
    };
    part("_floor", 0.0f, 0.0f, (kWall - z) * 0.5f, x, y, kWall);
    part("_north", 0.0f, (y - kWall) * 0.5f, 0.0f, x, kWall, z);
    part("_south", 0.0f, (kWall - y) * 0.5f, 0.0f, x, kWall, z);
    part("_east", (x - kWall) * 0.5f, 0.0f, 0.0f, kWall, y, z);
    part("_west", (kWall - x) * 0.5f, 0.0f, 0.0f, kWall, y, z);
}

bool inside(compages::world::Entity p_object, compages::world::Entity p_container)
{
    auto const at = p_object.position();
    auto const center = p_container.position();
    auto const& size = p_container.get<ecs::SceneObject>().size;
    return std::abs(at.x - center.x) < size[0] * 0.5f &&
           std::abs(at.y - center.y) < size[1] * 0.5f &&
           std::abs(at.z - center.z) < size[2] * 0.5f;
}

} // namespace

Simulation::Simulation(compages::world::World& p_world,
                       compages::renderer::Scene* p_scene,
                       Scenario p_scenario)
    : m_world(p_world), m_scenario(std::move(p_scenario))
{
    m_runtime = (p_scene != nullptr)
                    ? std::make_unique<RobotRuntime>(m_world, m_scenario.robot_model, *p_scene)
                    : std::make_unique<RobotRuntime>(m_world, m_scenario.robot_model);
    m_runtime->hold(m_scenario.home);
    m_context = std::make_unique<RobotContext>(m_runtime->context());
    spawn(p_scene);
    buildTree();
}

Simulation::~Simulation() = default;

void Simulation::spawn(compages::renderer::Scene* p_scene)
{
    m_world.each<ecs::RobotTag>([&](compages::world::Entity p_entity, ecs::RobotTag&)
                                { m_robot = p_entity; });

    compages::world::Entity tool = findTool(m_world);
    if (!tool)
    {
        throw std::runtime_error("The robot has no end effector to mount a gripper on");
    }
    tool.set(ecs::VacuumGripper{ {}, m_scenario.tool_length });

    for (Scenario::Object const& object : m_scenario.objects)
    {
        ecs::SceneObject const& shape = object.shape;
        compages::world::Entity body;
        if (p_scene != nullptr && shape.type == ecs::SceneObject::Type::Cube)
        {
            body = p_scene->box(shape.name, compages::renderer::color(
                                                shape.color[0], shape.color[1], shape.color[2]));
            body.scale(shape.size[0], shape.size[1], shape.size[2]);
        }
        else
        {
            body = m_world.entity(shape.name);
            if (p_scene != nullptr)
            {
                walls(*p_scene, body, shape);
            }
        }
        body.parent(m_robot)
            .position(object.position[0], object.position[1], object.position[2])
            .set(shape);
    }

    if (m_scenario.camera && p_scene != nullptr)
    {
        Scenario::Camera const& mount = *m_scenario.camera;
        compages::world::Entity link;
        m_world.each<ecs::Link>([&](compages::world::Entity p_entity, ecs::Link& p_link)
                                { link = (p_link.name == mount.link) ? p_entity : link; });
        if (!link)
        {
            throw std::runtime_error("Camera link '" + mount.link + "' not found");
        }
        compages::world::Entity active = p_scene->activeCamera();
        // Compages cameras look down their -Z: flip it onto the link +Z.
        m_camera = p_scene->camera("RobotCamera")
                       .parent(link)
                       .position(mount.position[0], mount.position[1], mount.position[2])
                       .rotation(std::numbers::pi_v<float>,
                                 compages::core::Vector3f(1.0f, 0.0f, 0.0f));
        auto& lens = m_camera.get<compages::world::Camera>();
        lens.fov = units::angle::degree_t(mount.sensor.fov_degrees);
        lens.near_plane = 0.01f;
        lens.far_plane = 20.0f;
        m_camera.set(mount.sensor).set(ecs::DetectedObjects{});
        if (active)
        {
            p_scene->activeCamera(active);
        }
    }
}

void Simulation::buildTree()
{
    auto add = [&](std::string const& p_name, std::shared_ptr<Skill> p_skill)
    { registerSkill(m_factory, p_name, std::move(p_skill), *m_context, m_trace); };

    add("Home", std::make_shared<HomeSkill>());
    add("Release", std::make_shared<ReleaseSkill>());
    for (Scenario::Object const& object : m_scenario.objects)
    {
        std::string const& name = object.shape.name;
        add("Detect(" + name + ")", std::make_shared<DetectSkill>(name));
        add("Approach(" + name + ")", std::make_shared<ApproachSkill>(name, kApproachClearance));
        add("Reach(" + name + ")", std::make_shared<ApproachSkill>(name, 0.0));
        add("Grasp(" + name + ")", std::make_shared<GraspSkill>(name));
    }

    if (m_scenario.behavior_tree.empty())
    {
        return;
    }
    m_blackboard = std::make_shared<bt::Blackboard>();
    auto built = bt::Builder::fromFile(m_factory, m_scenario.behavior_tree, m_blackboard);
    if (!built)
    {
        throw std::runtime_error("Behavior tree '" + m_scenario.behavior_tree +
                                 "': " + built.getError());
    }
    m_tree = std::move(built.getValue());
}

void Simulation::tick(double p_dt)
{
    m_context->time = m_runtime->time();
    m_context->dt = p_dt;
    if (m_tree && !finished())
    {
        m_status = m_tree->tick();
    }
    GraspSystem{}.update(m_world, m_runtime->kinematics());
    if (MujocoBackend* mujoco = m_runtime->simulation())
    {
        m_max_contacts = std::max(m_max_contacts, mujoco->contacts());
    }
}

void Simulation::step(double p_dt)
{
    tick(p_dt);
    m_runtime->step(p_dt);
}

void Simulation::step(compages::world::ViewFrame const& p_frame)
{
    tick(p_frame.elapsed);
    m_runtime->step(p_frame);
}

std::vector<Simulation::Check> Simulation::checks() const
{
    static std::regex const kInside(R"re(object\("([^"]+)"\)\.inside\("([^"]+)"\))re");

    std::vector<Check> result;
    for (std::string const& text : m_scenario.asserts)
    {
        Check check{ text, false };
        std::smatch match;
        if (text == "robot.success")
        {
            check.passed = m_status == bt::Status::SUCCESS;
        }
        else if (text == "gripper.empty")
        {
            compages::world::Entity tool = findTool(m_world);
            check.passed = tool && !m_world.alive(tool.get<ecs::VacuumGripper>().held);
        }
        else if (text == "collisions == 0")
        {
            check.passed = m_max_contacts == 0;
        }
        else if (std::regex_match(text, match, kInside))
        {
            compages::world::Entity object = findObject(m_world, match[1].str());
            compages::world::Entity container = findObject(m_world, match[2].str());
            check.passed = object && container && inside(object, container);
        }
        else
        {
            check.text += "  (unknown assertion)";
        }
        result.push_back(std::move(check));
    }
    return result;
}

} // namespace robotik
