// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "Robotik/Runtime/Simulation.hpp"

#include "Robotik/Backends/MujocoBackend.hpp"
#include "Robotik/Behavior/SkillNodes.hpp"
#include "Robotik/ECS/Queries.hpp"
#include "Robotik/Skills/PickPlaceSkills.hpp"
#include "Robotik/Systems/GraspSystem.hpp"

#include "Compages/World/World.hpp"

#include <algorithm>
#include <cmath>
#include <regex>
#include <stdexcept>

#define APPROACH_CLEARANCE_M 0.10
#define SKILL_PRIORITY 100
#define STOP_PRIORITY 1000
#define WORLD_MODEL_GATE_M 0.10
#define OBJECT_INSIDE_ASSERT_REGEX \
    R"re(object\("([^"]+)"\)\.inside\("([^"]+)"\))re"

namespace robotik
{

static Vector3 positionOf(compages::world::Entity p_entity)
{
    auto const at = p_entity.position();
    return { at.x, at.y, at.z };
}

static Vector3 sizeOf(ecs::SceneObject const& p_object)
{
    return { p_object.size[0].value(),
             p_object.size[1].value(),
             p_object.size[2].value() };
}

static bool inside(compages::world::Entity p_object,
                   compages::world::Entity p_container)
{
    Vector3 const at = positionOf(p_object);
    Vector3 const center = positionOf(p_container);
    Vector3 const size = sizeOf(p_container.get<ecs::SceneObject>());
    return std::abs(at.x - center.x) < size.x * 0.5 &&
           std::abs(at.y - center.y) < size.y * 0.5 &&
           std::abs(at.z - center.z) < size.z * 0.5;
}

Simulation::Simulation(compages::world::World& p_world,
                       Scenario p_scenario,
                       SceneView* p_view)
    : m_world(p_world),
      m_scenario(std::move(p_scenario)),
      m_robot(std::make_unique<RobotSession>(
          p_world, m_scenario.robot_model, p_view)),
      m_scheduler(std::make_unique<SkillScheduler>(m_robot->resources())),
      m_faults(m_scenario.faults, m_scenario.random_faults),
      m_context{ *m_robot, m_world_model, {}, {} }
{
    m_robot->connect(std::make_unique<MujocoBackend>(m_scenario.robot_model));
    m_robot->hold(m_scenario.home);
    spawn(p_view);
    if (!m_oracle)
    {
        m_world_model.gate(WORLD_MODEL_GATE_M);
    }
    addSkills();
    registerSkills(m_factory, *m_scheduler);
    reset();
}

Simulation::~Simulation() = default;

void Simulation::spawn(SceneView* p_view)
{
    ActuatorSet& actuators = m_robot->actuators();
    if (m_scenario.actuators.empty())
    {
        actuators.add<JointGroup>("arm");
        actuators.add<VacuumGripper>("gripper");
    }
    for (Scenario::Actuator const& actuator : m_scenario.actuators)
    {
        switch (actuator.type)
        {
            case Scenario::Actuator::Type::JointGroup:
                actuators.add<JointGroup>(actuator.name, actuator.joints);
                break;
            case Scenario::Actuator::Type::Motor:
                actuators.add<Motor>(actuator.name, actuator.joints.front());
                break;
            case Scenario::Actuator::Type::Vacuum:
                actuators.add<VacuumGripper>(
                    actuator.name, actuator.link, actuator.length);
                break;
        }
    }

    for (Scenario::Camera const& spec : m_scenario.cameras)
    {
        Camera& camera = m_robot->sensors().add<Camera>(spec.name, spec.config);
        compages::world::Entity link =
            spec.config.parent.empty() ? m_robot->root()
                                       : m_robot->link(spec.config.parent);
        if (!link)
        {
            throw std::runtime_error("Camera '" + spec.name +
                                     "': unknown link '" + spec.config.parent +
                                     "'");
        }
        if (p_view != nullptr)
        {
            camera.source(p_view->camera(camera, link));
        }
        m_oracle = m_oracle && camera.source() == nullptr;
        camera.onFrame([this](CameraFrame const& p_frame)
                       { m_world_model.update(m_perception.process(p_frame)); });
    }

    for (Scenario::Object const& object : m_scenario.objects)
    {
        compages::world::Entity entity = m_world.entity(object.shape.name);
        entity.parent(m_robot->root()).set(object.shape);
        if (p_view != nullptr)
        {
            p_view->object(entity, object.shape);
        }
        m_objects.push_back(entity);
    }
}

void Simulation::addSkills()
{
    ResourceManager& resources = m_robot->resources();
    ActuatorSet const& actuators = m_robot->actuators();

    std::vector<ResourceRequirement> arm;
    if (auto const* group = actuators.first<JointGroup>())
    {
        arm.push_back(resources.require(group->name()));
    }
    std::vector<ResourceRequirement> gripper;
    if (auto const* vacuum = actuators.first<VacuumGripper>())
    {
        gripper.push_back(resources.require(vacuum->name()));
    }
    std::vector<ResourceRequirement> camera;
    if (Camera const* found = this->camera())
    {
        camera.push_back(resources.require(found->name(), Access::Shared));
    }
    std::vector<ResourceRequirement> everything;
    for (std::size_t i = 0; i < actuators.size(); ++i)
    {
        everything.push_back({ actuators.resource(i), Access::Exclusive });
    }

    auto describe = [](std::string p_name,
                       std::vector<ResourceRequirement> p_resources)
    {
        SkillDescription description;
        description.name = std::move(p_name);
        description.resources = std::move(p_resources);
        description.priority = SKILL_PRIORITY;
        return description;
    };

    SkillScheduler& skills = *m_scheduler;
    skills.add<HomeSkill>(describe("Home", arm));
    skills.add<ReleaseSkill>(describe("Release", gripper));
    SkillDescription stop = describe("Stop", everything);
    stop.priority = STOP_PRIORITY;
    skills.add<StopSkill>(std::move(stop));

    for (Scenario::Object const& object : m_scenario.objects)
    {
        std::string const& name = object.shape.name;
        skills.add<DetectSkill>(describe("Detect(" + name + ")", camera), name);
        skills.add<ApproachSkill>(describe("Approach(" + name + ")", arm),
                                  name,
                                  Length(APPROACH_CLEARANCE_M));
        skills.add<ApproachSkill>(
            describe("Reach(" + name + ")", arm), name, Length{});

        SkillDescription grasp = describe("Grasp(" + name + ")", gripper);
        grasp.wait = false;
        grasp.preconditions.push_back(
            { "gripper is empty",
              [](RobotContext const& p_context)
              {
                  VacuumGripper const* vacuum =
                      findGripper(p_context.robot, std::string{});
                  return vacuum != nullptr && !vacuum->holding();
              } });
        skills.add<GraspSkill>(std::move(grasp), name);
    }
}

void Simulation::loadTree()
{
    m_tree.reset();
    m_status = bt::Status::INVALID;
    if (m_scenario.behavior_tree.empty())
    {
        return;
    }
    m_blackboard = std::make_shared<bt::Blackboard>();
    auto built = bt::Builder::fromFile(
        m_factory, m_scenario.behavior_tree.string(), m_blackboard);
    if (!built)
    {
        throw std::runtime_error("Behavior tree '" +
                                 m_scenario.behavior_tree.string() +
                                 "': " + built.getError());
    }
    m_tree = std::move(built.getValue());
}

void Simulation::reset(Seed p_seed)
{
    m_seed = p_seed;
    m_max_contacts = 0;

    m_robot->resources().restoreAll();
    m_scheduler->reset();
    m_faults.reset(p_seed.derive("faults"));
    ActuatorSet& actuators = m_robot->actuators();
    for (std::size_t i = 0; i < actuators.size(); ++i)
    {
        if (auto* vacuum = dynamic_cast<VacuumGripper*>(&actuators[i]))
        {
            vacuum->suction(false);
            vacuum->held({});
        }
    }
    SensorSet& sensors = m_robot->sensors();
    for (std::size_t i = 0; i < sensors.size(); ++i)
    {
        if (auto* camera = dynamic_cast<Camera*>(&sensors[i]))
        {
            camera->seed(p_seed.derive(camera->name()));
        }
    }

    // The mission knows the nominal layout; the actual one is randomized.
    Random random(p_seed.derive("world"));
    m_world_model.clear();
    for (std::size_t i = 0; i < m_objects.size(); ++i)
    {
        Scenario::Object const& object = m_scenario.objects[i];
        Vector3 at = object.position;
        at.x += random.uniform(object.randomize[0][0], object.randomize[0][1]);
        at.y += random.uniform(object.randomize[1][0], object.randomize[1][1]);
        at.z += random.uniform(object.randomize[2][0], object.randomize[2][1]);
        m_objects[i].position(static_cast<float>(at.x),
                              static_cast<float>(at.y),
                              static_cast<float>(at.z));
        m_world_model.add(object.shape.name, object.position, sizeOf(object.shape));
    }

    m_robot->reset();
    loadTree();
}

Camera* Simulation::camera() const
{
    return m_robot->sensors().first<Camera>();
}

void Simulation::observe()
{
    SensorSet const& sensors = m_robot->sensors();
    bool sees = sensors.size() == 0u;
    for (std::size_t i = 0; i < sensors.size() && !sees; ++i)
    {
        sees = sensors.available(i);
    }
    if (!sees)
    {
        return;
    }
    for (compages::world::Entity const& entity : m_objects)
    {
        (void)m_world_model.observe(entity.get<ecs::SceneObject>().name,
                                    positionOf(entity),
                                    1.0f,
                                    m_robot->time());
    }
}

void Simulation::step(Seconds p_dt)
{
    m_faults.update(m_robot->resources(), m_robot->time(), p_dt);

    m_context.time = m_robot->time();
    m_context.dt = p_dt;
    if (m_tree && !finished())
    {
        m_status = m_tree->tick();
    }
    m_scheduler->update(m_context);

    m_robot->step(p_dt);
    GraspSystem{}.update(*m_robot);
    if (m_oracle)
    {
        observe();
    }
    if (RobotBackend const* backend = m_robot->backend())
    {
        m_max_contacts = std::max(m_max_contacts, backend->contacts());
    }
}

std::vector<Simulation::Check> Simulation::checks() const
{
    static std::regex const inside_assert_regex(OBJECT_INSIDE_ASSERT_REGEX);

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
            VacuumGripper const* vacuum = findGripper(*m_robot, std::string{});
            check.passed = vacuum != nullptr && !vacuum->holding();
        }
        else if (text == "collisions == 0")
        {
            check.passed = m_max_contacts == 0;
        }
        else if (std::regex_match(text, match, inside_assert_regex))
        {
            compages::world::Entity object = findObject(m_world, match[1].str());
            compages::world::Entity container =
                findObject(m_world, match[2].str());
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
