// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "Robotik/Runtime/Simulation.hpp"

#include "Robotik/Backends/MujocoBackend.hpp"
#include "Robotik/ECS/ObjectComponents.hpp"
#include "Robotik/ECS/Queries.hpp"
#include "Robotik/Robot/Actuators.hpp"
#include "Robotik/Scene/ContainerBounds.hpp"
#include "Robotik/Sensors/ForceTorqueSensor.hpp"
#include "Robotik/Sensors/Imu.hpp"
#include "Robotik/Sensors/RangeScanner.hpp"
#include "Robotik/Skills/MotionSkills.hpp"
#include "Robotik/Skills/SkillNodes.hpp"
#include "Robotik/Systems/GraspSystem.hpp"

#include "Compages/World/World.hpp"

#include <algorithm>
#include <cmath>
#include <optional>
#include <regex>
#include <stdexcept>

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

//! While the vacuum holds a prop, flag wall penetration (counts toward
//! @c collisions). Sticky until @ref Simulation::reset.
static void trackPropPenetration(RobotSession& p_robot, bool& p_flag)
{
    if (p_flag)
    {
        return;
    }

    VacuumGripper const* gripper = p_robot.actuators().first<VacuumGripper>();
    if (gripper == nullptr || !gripper->holding())
    {
        return;
    }

    compages::world::World& world = p_robot.world();
    if (!world.alive(gripper->held()))
    {
        return;
    }

    compages::world::Entity const held = world.entity(gripper->held());
    ecs::SceneObject const* prop = held.find<ecs::SceneObject>();
    if (prop == nullptr)
    {
        return;
    }
    Vector3 const at = positionOf(held);
    Vector3 const half = scene::halfExtents(*prop);

    world.each<ecs::SceneObject>(
        [&](compages::world::Entity p_entity, ecs::SceneObject const& p_object)
        {
            if (p_flag || p_object.type != ecs::SceneObject::Type::BOX ||
                p_entity.id() == held.id())
            {
                return;
            }
            if (scene::penetratesContainer(
                    p_object, positionOf(p_entity), at, half))
            {
                p_flag = true;
            }
        });
}

//! @c object("…").inside("…"): resting in an open box, otherwise inside its
//! AABB.
static bool inside(compages::world::Entity p_object,
                   compages::world::Entity p_container)
{
    Vector3 const at = positionOf(p_object);
    Vector3 const center = positionOf(p_container);
    ecs::SceneObject const& container = p_container.get<ecs::SceneObject>();
    ecs::SceneObject const& object = p_object.get<ecs::SceneObject>();

    if (container.type == ecs::SceneObject::Type::BOX)
    {
        return scene::restsInside(
            container, center, at, scene::halfExtents(object));
    }

    Vector3 const size = sizeOf(container);
    return std::abs(at.x - center.x) < size.x * 0.5 &&
           std::abs(at.y - center.y) < size.y * 0.5 &&
           std::abs(at.z - center.z) < size.z * 0.5;
}

Simulation::Simulation(compages::world::World& p_world,
                       Scenario p_scenario,
                       SceneView* p_view,
                       Mission* p_mission)
    : m_world(p_world),
      m_scenario(std::move(p_scenario)),
      m_mission(p_mission),
      m_robot(std::make_unique<RobotSession>(p_world,
                                             m_scenario.robot_model,
                                             p_view)),
      m_scheduler(std::make_unique<SkillScheduler>(m_robot->resources())),
      m_faults(m_scenario.faults, m_scenario.random_faults),
      m_context{ *m_robot, m_world_model, {}, {} }
{
    // Spawn the scenario into ECS, backend, and generic skills.
    spawn(p_view);

    // Setup the mission.
    if (m_mission != nullptr)
    {
        m_mission->setup(*this, p_view);
    }

    // Connect the backend.
    if (m_robot->backend() == nullptr)
    {
        m_robot->connect(
            std::make_unique<MujocoBackend>(m_scenario.robot_model));
    }

    // Hold the home position.
    m_robot->hold(m_scenario.home);

    // Set the world model gate.
    if (!m_oracle)
    {
        m_world_model.gate(WORLD_MODEL_GATE_M);
    }

    // Add and register the skills.
    addSkills();
    registerSkills(m_factory, *m_scheduler);

    reset();
}

Simulation::~Simulation() = default;

void Simulation::spawn(SceneView* p_view)
{
    // Actuators from scenario (or arm + vacuum defaults).
    ActuatorSet& actuators = m_robot->actuators();
    if (m_scenario.actuators.empty())
    {
        actuators.add<JointGroup>("arm");
        actuators.add<VacuumGripper>("gripper");
    }

    // Add the actuators.
    for (Scenario::Actuator const& actuator : m_scenario.actuators)
    {
        // Add the actuator based on its type.
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

    // Cameras: render source in the simulator, or oracle mode when headless.
    for (Scenario::Camera const& spec : m_scenario.cameras)
    {
        Camera& camera = m_robot->sensors().add<Camera>(spec.name, spec.config);

        // Get the link for the camera.
        compages::world::Entity link = spec.config.parent.empty()
                                           ? m_robot->root()
                                           : m_robot->link(spec.config.parent);
        if (!link)
        {
            throw std::runtime_error("Camera '" + spec.name +
                                     "': unknown link '" + spec.config.parent +
                                     "'");
        }

        // Set the camera source.
        if (p_view != nullptr)
        {
            camera.source(p_view->camera(camera, link));
        }

        // Set the world model gate.
        m_oracle = m_oracle && camera.source() == nullptr;

        // Set the camera on frame callback.
        camera.onFrame(
            [this](CameraFrame const& p_frame)
            { m_world_model.update(m_perception.process(p_frame)); });
    }

    // Props: @ref ecs::SceneObject on entities parented to the robot base
    // frame.
    for (Scenario::Object const& object : m_scenario.objects)
    {
        // Get the entity for the object.
        compages::world::Entity entity = m_world.entity(object.shape.name);

        // Parent the entity to the robot base frame.
        entity.parent(m_robot->root()).set(object.shape);

        // Set the object on view callback.
        if (p_view != nullptr)
        {
            p_view->object(entity, object.shape);
        }
        m_objects.push_back(entity);
    }
}

void Simulation::addSkills()
{
    ResourceManager const& resources = m_robot->resources();
    ActuatorSet const& actuators = m_robot->actuators();

    // Add the arm skill.
    std::vector<ResourceRequirement> arm;
    if (auto const* group = actuators.first<JointGroup>())
    {
        arm.push_back(resources.require(group->name()));
    }

    // Add the everything resource requirement.
    std::vector<ResourceRequirement> everything;
    for (std::size_t i = 0; i < actuators.size(); ++i)
    {
        everything.push_back({ actuators.resource(i), Access::Exclusive });
    }

    // Describe the skill.
    auto describe =
        [](std::string p_name, std::vector<ResourceRequirement> p_resources)
    {
        SkillDescription description;
        description.name = std::move(p_name);
        description.resources = std::move(p_resources);
        description.priority = SKILL_PRIORITY;
        return description;
    };

    // Add the skills.
    SkillScheduler& skills = *m_scheduler;
    if (!arm.empty())
    {
        skills.add<HomeSkill>(describe("Home", arm));
    }

    // Add the stop skill.
    SkillDescription stop = describe("Stop", everything);
    stop.priority = STOP_PRIORITY;
    if (!everything.empty())
    {
        skills.add<StopSkill>(std::move(stop));
    }
}

void Simulation::loadTree()
{
    // Reset the behavior tree.
    m_tree.reset();
    m_status = bt::Status::INVALID;
    if (m_scenario.behavior_tree.empty())
    {
        return;
    }

    // Build the behavior tree with its blackboard.
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
    m_prop_penetration = false;

    m_robot->resources().restoreAll();
    m_scheduler->reset();
    m_faults.reset(p_seed.derive("faults"));

    // Reset the actuators.
    ActuatorSet const& actuators = m_robot->actuators();
    for (std::size_t i = 0; i < actuators.size(); ++i)
    {
        if (auto* vacuum = dynamic_cast<VacuumGripper*>(&actuators[i]))
        {
            vacuum->suction(false);
            vacuum->held({});
        }
    }

    // Seed the sensors.
    SensorSet const& sensors = m_robot->sensors();
    for (std::size_t i = 0; i < sensors.size(); ++i)
    {
        if (auto* camera = dynamic_cast<Camera*>(&sensors[i]))
        {
            camera->seed(p_seed.derive(camera->name()));
        }
        else if (auto* imu = dynamic_cast<Imu*>(&sensors[i]))
        {
            imu->seed(p_seed.derive(imu->name()));
        }
        else if (auto* scanner = dynamic_cast<RangeScanner*>(&sensors[i]))
        {
            scanner->seed(p_seed.derive(scanner->name()));
        }
        else if (auto* force = dynamic_cast<ForceTorqueSensor*>(&sensors[i]))
        {
            force->seed(p_seed.derive(force->name()));
        }
    }

    // Nominal layout for perception; true poses are randomized on the entities.
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
        m_world_model.add(
            object.shape.name, object.position, sizeOf(object.shape));
    }

    m_robot->reset();
    loadTree();
    if (m_mission != nullptr)
    {
        m_mission->reset(*this, p_seed);
    }
}

Camera* Simulation::camera() const
{
    return m_robot->sensors().first<Camera>();
}

void Simulation::observe()
{
    // Check if any sensors are available.
    SensorSet const& sensors = m_robot->sensors();
    bool sees = sensors.size() == 0u;
    for (std::size_t i = 0; i < sensors.size() && !sees; ++i)
    {
        sees = sensors.available(i);
    }

    // No sensors are available.
    if (!sees)
    {
        return;
    }

    // Observe the objects.
    for (compages::world::Entity const& entity : m_objects)
    {
        m_world_model.place(entity.get<ecs::SceneObject>().name,
                            positionOf(entity),
                            m_robot->time());
    }
}

void Simulation::step(Seconds p_dt)
{
    // Update the faults.
    m_faults.update(m_robot->resources(), m_robot->time(), p_dt);

    // Update the context.
    m_context.time = m_robot->time();
    m_context.dt = p_dt;

    // Tick the behavior tree.
    if (m_tree && !finished())
    {
        m_status = m_tree->tick();
    }
    m_scheduler->update(m_context);

    // MuJoCo arm, then kinematic props (vacuum)
    m_robot->step(p_dt);
    GraspSystem{}.update(*m_robot);
    trackPropPenetration(*m_robot, m_prop_penetration);

    // Observe the objects if in oracle mode.
    if (m_oracle)
    {
        observe();
    }

    // Update the maximum contacts.
    if (RobotBackend const* backend = m_robot->backend())
    {
        m_max_contacts = std::max(m_max_contacts, backend->contacts());
    }

    // Update the mission.
    if (m_mission != nullptr)
    {
        m_mission->step(*this, p_dt);
        if (m_tree == nullptr)
        {
            switch (m_mission->status(*this))
            {
                case Status::SUCCESS:
                    m_status = bt::Status::SUCCESS;
                    break;
                case Status::FAILURE:
                    m_status = bt::Status::FAILURE;
                    break;
                default:
                    break;
            }
        }
    }
}

std::vector<Check> Simulation::checks() const
{
    static std::regex const inside_assert_regex(OBJECT_INSIDE_ASSERT_REGEX);

    Metrics metrics;
    metrics.set("time", time().value());
    metrics.set(
        "collisions",
        static_cast<double>(m_max_contacts + (m_prop_penetration ? 1 : 0)));
    metrics.set("robot.success", m_status == bt::Status::SUCCESS ? 1.0 : 0.0);
    VacuumGripper const* vacuum = m_robot->actuators().first<VacuumGripper>();
    if (vacuum != nullptr)
    {
        metrics.set("gripper.empty", vacuum->holding() ? 0.0 : 1.0);
    }
    if (m_mission != nullptr)
    {
        m_mission->measure(*this, metrics);
    }

    MetricResolver resolver =
        [this](std::string_view p_name) -> std::optional<double>
    {
        std::smatch match;
        std::string const text(p_name);
        if (std::regex_match(text, match, inside_assert_regex))
        {
            compages::world::Entity object =
                findObject(m_world, match[1].str());
            compages::world::Entity container =
                findObject(m_world, match[2].str());
            return object && container && inside(object, container) ? 1.0 : 0.0;
        }
        return std::nullopt;
    };

    // Evaluate the asserts.
    std::vector<Check> result;
    result.reserve(m_scenario.asserts.size());
    for (std::string const& text : m_scenario.asserts)
    {
        result.push_back(evaluate(text, metrics, resolver));
    }
    return result;
}

} // namespace robotik
