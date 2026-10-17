// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "Robotik/Runtime/Simulation.hpp"
#include "ShapeOf.hpp"

#include "Robotik/Backends/MujocoBackend.hpp"
#include "Robotik/Backends/PinocchioBackend.hpp"
#include "Robotik/ECS/ObjectComponents.hpp"
#include "Robotik/Robot/Actuators.hpp"
#include "Robotik/Scene/ContainerBounds.hpp"
#include "Robotik/Sensors/ForceTorqueSensor.hpp"
#include "Robotik/Sensors/Imu.hpp"
#include "Robotik/Sensors/RangeScanner.hpp"
#include "Robotik/Skills/MotionSkills.hpp"
#include "Robotik/Skills/SkillNodes.hpp"
#include "Robotik/Systems/GraspModel.hpp"

#include "Compages/World/World.hpp"

#include <pugixml.hpp>

#include <algorithm>
#include <chrono>
#include <stdexcept>
#include <string>

#define SKILL_PRIORITY 100
#define STOP_PRIORITY 1000
#define WORLD_MODEL_GATE_M 0.10
//! Name of the Z-up frame entity the world entities hang from.
#define WORLD_FRAME_ENTITY "robotik.world"

namespace robotik
{

namespace
{

Vector3 positionOf(compages::world::Entity p_entity)
{
    auto const at = p_entity.position();
    return { at.x, at.y, at.z };
}

compages::core::Vector3f toFloat(Vector3 const& p_v)
{
    return { static_cast<float>(p_v.x), static_cast<float>(p_v.y), static_cast<float>(p_v.z) };
}

compages::core::Quatf toFloat(Quaternion const& p_q)
{
    return { static_cast<float>(p_q.w),
             static_cast<float>(p_q.x),
             static_cast<float>(p_q.y),
             static_cast<float>(p_q.z) };
}

//! Links of a URDF and its root (the link no joint has as child).
struct UrdfLinks
{
    std::string root;
    std::unordered_set<std::string> names;
};

UrdfLinks readLinks(std::filesystem::path const& p_urdf)
{
    pugi::xml_document document;
    if (!document.load_file(p_urdf.string().c_str()))
    {
        throw std::runtime_error("Cannot parse URDF '" + p_urdf.string() + "'");
    }
    pugi::xml_node const robot = document.child("robot");
    UrdfLinks links;
    std::unordered_set<std::string> children;
    for (pugi::xml_node joint : robot.children("joint"))
    {
        children.insert(joint.child("child").attribute("link").as_string());
    }
    for (pugi::xml_node link : robot.children("link"))
    {
        std::string name = link.attribute("name").as_string();
        if (links.root.empty() && !children.contains(name))
        {
            links.root = name;
        }
        links.names.insert(std::move(name));
    }
    return links;
}

//! Solid box inertia at the centre (kg.m^2).
Vector3 boxInertia(double p_mass, Vector3 const& p_size)
{
    double const x2 = p_size.x * p_size.x;
    double const y2 = p_size.y * p_size.y;
    double const z2 = p_size.z * p_size.z;
    return { p_mass * (y2 + z2) / 12.0, p_mass * (x2 + z2) / 12.0, p_mass * (x2 + y2) / 12.0 };
}

//! While the vacuum holds a prop, flag wall penetration (counts as a robot
//! collision). Sticky until @ref Simulation::reset.
void trackPropPenetration(RobotSession& p_robot, bool& p_flag)
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
    ecs::Shape const* prop = held.find<ecs::Shape>();
    if (prop == nullptr || held.find<ecs::PhysicsBody>() != nullptr)
    {
        return; // A rigid body: the physics sees its contacts.
    }
    Vector3 const at = positionOf(held);
    Vector3 const half = scene::halfExtents(*prop);
    world.each<ecs::Shape>(
        [&](compages::world::Entity p_entity, ecs::Shape const& p_object)
        {
            if (p_flag || p_object.type != ecs::Shape::Type::CONTAINER ||
                p_entity.id() == held.id() || p_entity.find<ecs::PhysicsBody>() != nullptr ||
                p_entity.find<ecs::Decoration>() != nullptr)
            {
                return;
            }
            if (scene::penetratesContainer(p_object, positionOf(p_entity), at, half))
            {
                p_flag = true;
            }
        });
}

} // namespace

char const* toString(Simulation::Outcome p_outcome)
{
    switch (p_outcome)
    {
        case Simulation::Outcome::Running:
            return "running";
        case Simulation::Outcome::ControllerSucceeded:
            return "controller succeeded";
        case Simulation::Outcome::ControllerFailed:
            return "controller failed";
        case Simulation::Outcome::StopCondition:
            return "stop condition";
        case Simulation::Outcome::Timeout:
            return "timeout";
        case Simulation::Outcome::AssertionFailed:
            return "assertion failed";
        case Simulation::Outcome::Interrupted:
            return "interrupted";
    }
    return "?";
}

Simulation::Simulation(compages::world::World& p_world,
                       Scenario p_scenario,
                       SceneView* p_view,
                       Mission* p_mission)
    : m_world(p_world),
      m_scenario(std::move(p_scenario)),
      m_mission(p_mission),
      m_view(p_view),
      m_robot([&]() {
          WorldDescription const& world = m_scenario.world;
          EntitySpec const* tool = world.mountedTool();
          return std::make_unique<RobotSession>(
              p_world, world.robot().model, p_view, tool != nullptr ? tool->model : std::filesystem::path{});
      }()),
      m_scheduler(std::make_unique<SkillScheduler>(m_robot->resources())),
      m_faults(m_scenario.faults, m_scenario.random_faults),
      m_context{ *m_robot, m_world_model, {}, {} },
      m_assertions(m_scenario.assertions)
{
    // Robot devices, props and bodies of the world file.
    build(p_view);

    // The mission adds its skills, detectors and dressing; it may also plug
    // its own backend.
    if (m_mission != nullptr)
    {
        m_mission->setup(*this, p_view);
    }
    if (m_robot->backend() == nullptr)
    {
        connectBackend();
    }

    // Hold the home position, now with the backend.
    m_robot->hold(m_scenario.world.robot().home);
    if (!m_oracle)
    {
        m_world_model.gate(WORLD_MODEL_GATE_M);
    }

    addSkills();
    registerSkills(m_factory, *m_scheduler);
    if (m_scenario.controller.planner)
    {
        m_executor = std::make_unique<PlanExecutor>(*m_scenario.controller.planner,
                                                    m_scenario.world, m_planning, m_scenario.id,
                                                    m_factory, *m_scheduler);
        m_executor->clock(simulatedClock());
    }
    reset();
}

Simulation::~Simulation() = default;

void Simulation::build(SceneView* p_view)
{
    // Objects without physics; the rigid ones get their model with the backend.
    m_grasps.clear();
    m_grasps.push_back(std::make_unique<AbstractGrasp>());

    WorldDescription const& world = m_scenario.world;
    EntitySpec const& robot = world.robot();
    EntitySpec const* tool = world.mountedTool();

    m_robot->startPose(robot.pose);
    // Robot frames of the attachments are taken at home.
    m_robot->hold(robot.home);

    // --- Actuators: the robot's, then those of the mounted tool. ------------
    ActuatorSet& actuators = m_robot->actuators();
    std::vector<ActuatorSpec> specs = robot.actuators;
    if (tool != nullptr)
    {
        specs.insert(specs.end(), tool->actuators.begin(), tool->actuators.end());
    }
    for (ActuatorSpec const& actuator : specs)
    {
        switch (actuator.type)
        {
            case ActuatorSpec::Type::JointGroup:
                actuators.add<JointGroup>(actuator.name, actuator.joints);
                break;
            case ActuatorSpec::Type::Motor:
                actuators.add<Motor>(actuator.name, actuator.joints.front());
                break;
            case ActuatorSpec::Type::Vacuum:
                actuators.add<VacuumGripper>(actuator.name, actuator.link, actuator.length);
                break;
        }
    }

    // --- Sensors: cameras get a render source in the simulator. -------------
    std::vector<SensorSpec> sensors = robot.sensors;
    if (tool != nullptr)
    {
        sensors.insert(sensors.end(), tool->sensors.begin(), tool->sensors.end());
    }
    bool blind = true;
    auto requireFrame = [this](SensorSpec const& p_spec, std::string const& p_frame)
    {
        if (!p_frame.empty() && !m_robot->link(p_frame))
        {
            throw std::runtime_error("Sensor '" + p_spec.name + "': unknown link '" + p_frame +
                                     "'");
        }
    };
    for (SensorSpec const& spec : sensors)
    {
        switch (spec.type)
        {
            case SensorSpec::Type::Imu:
                requireFrame(spec, spec.imu.parent);
                m_robot->sensors().add<Imu>(spec.name, spec.imu);
                continue;
            case SensorSpec::Type::Range:
                requireFrame(spec, spec.range.parent);
                m_robot->sensors().add<RangeScanner>(spec.name, spec.range);
                continue;
            case SensorSpec::Type::ForceTorque:
                requireFrame(spec, spec.force_torque.link);
                m_robot->sensors().add<ForceTorqueSensor>(spec.name, spec.force_torque);
                continue;
            case SensorSpec::Type::Camera:
                break;
        }
        requireFrame(spec, spec.camera.parent);
        Camera& camera = m_robot->sensors().add<Camera>(spec.name, spec.camera);
        compages::world::Entity link = spec.camera.parent.empty()
                                           ? m_robot->root()
                                           : m_robot->link(spec.camera.parent);
        if (p_view != nullptr)
        {
            camera.source(p_view->camera(camera, link));
        }
        blind = blind && camera.source() == nullptr;
        camera.onFrame([this](CameraFrame const& p_frame)
                       { m_world_model.update(m_perception.process(p_frame)); });
    }
    m_oracle = m_scenario.oracle == OraclePolicy::Always ||
               (m_scenario.oracle == OraclePolicy::WhenBlind && blind);

    // --- Entities: one record each, in file order. ---------------------------
    m_frame = m_world.entity(WORLD_FRAME_ENTITY);
    m_frame.rotation(m_robot->urdfFrame());
    m_records.reserve(world.entities.size());
    for (EntitySpec const& spec : world.entities)
    {
        if (spec.type == EntitySpec::Type::Tool && !spec.attach)
        {
            continue; // In storage.
        }
        Record& record = m_records.emplace_back();
        record.spec = &spec;
        buildRecord(record, p_view);
    }
}

void Simulation::buildRecord(Record& p_record, SceneView* p_view)
{
    EntitySpec const& spec = *p_record.spec;
    switch (spec.type)
    {
        case EntitySpec::Type::Robot:
            p_record.links = readLinks(spec.model).names;
            p_record.entity = m_robot->root();
            p_record.nominal = spec.pose;
            return;
        case EntitySpec::Type::Tool:
            p_record.links = readLinks(spec.model).names;
            return;
        case EntitySpec::Type::Prop:
        {
            ecs::Shape const object = shapeOf(spec);
            p_record.entity = m_world.entity(spec.id);
            p_record.entity.parent(m_frame).set(object);
            if (rigidProp(spec))
            {
                // A body of the physics backend: the abstract grasp leaves it
                // alone. A box moves (free body), a container does not (walls).
                p_record.entity.set(ecs::PhysicsBody{ spec.id });
                if (object.type == ecs::Shape::Type::BOX)
                {
                    p_record.body = spec.id;
                    p_record.links = { spec.id };
                }
                else
                {
                    p_record.links = { spec.id + "_floor", spec.id + "_wall_x0", spec.id + "_wall_x1",
                                       spec.id + "_wall_y0", spec.id + "_wall_y1" };
                }
            }
            if (p_view != nullptr)
            {
                p_view->object(p_record.entity, object);
            }
            p_record.nominal = resolveNominal(spec);
            return;
        }
        case EntitySpec::Type::Body:
            break;
    }

    p_record.nominal = resolveNominal(spec);
    if (spec.shape)
    {
        p_record.body = spec.id;
        p_record.links = { spec.id };
        ecs::Shape const object = shapeOf(spec);
        p_record.entity = m_world.entity(spec.id);
        p_record.entity.parent(m_frame).set(object).set(ecs::PhysicsBody{ spec.id });
        if (p_view != nullptr)
        {
            p_view->object(p_record.entity, object);
        }
        return;
    }
    UrdfLinks links = readLinks(spec.model);
    p_record.body = links.root;
    p_record.links = std::move(links.names);
    p_record.frames = std::make_shared<PinocchioBackend>(spec.model);
    if (p_view != nullptr)
    {
        p_record.entity = p_view->model(m_world, spec.model);
    }
}

Pose Simulation::resolveNominal(EntitySpec const& p_spec) const
{
    if (!p_spec.attach)
    {
        return p_spec.pose;
    }
    Attachment const& attach = *p_spec.attach;
    EntitySpec const* parent = m_scenario.world.find(attach.parent);
    Pose base{ zero3(), Quaternion{} };
    if (parent->type == EntitySpec::Type::Robot)
    {
        base = m_robot->worldPose(attach.parent_frame);
    }
    else
    {
        Record const* record = this->record(parent->id);
        if (record == nullptr)
        {
            throw std::runtime_error("Entity '" + p_spec.id + "' hangs from '" + parent->id +
                                     "', declared after it");
        }
        base = record->nominal;
        if (!attach.parent_frame.empty())
        {
            if (record->frames == nullptr || !record->frames->hasFrame(attach.parent_frame))
            {
                throw std::runtime_error("Entity '" + p_spec.id + "': '" + parent->id +
                                         "' has no frame '" + attach.parent_frame + "'");
            }
            base = base * record->frames->framePose(attach.parent_frame);
        }
    }
    return base * attach.transform;
}

void Simulation::connectBackend()
{
    WorldDescription const& world = m_scenario.world;
    MujocoOptions options;
    // Rigid props rest on the ground: they need a floor.
    bool const rigid = std::any_of(world.entities.begin(), world.entities.end(),
                                   [this](EntitySpec const& p_entity)
                                   { return p_entity.type == EntitySpec::Type::Prop && rigidProp(p_entity); });
    options.floor = world.floor || rigid;
    options.floor_friction = world.floor_friction;
    auto physics = std::make_unique<MujocoBackend>(options);
    std::string const arm = physics->load(world.robot().model);
    if (EntitySpec const* tool = world.mountedTool())
    {
        std::string const mounted = physics->load(tool->model);
        physics->attach(arm, "flange", mounted, "tool_mount");
    }
    for (Record const& record : m_records)
    {
        EntitySpec const& spec = *record.spec;
        if (spec.type != EntitySpec::Type::Body)
        {
            continue;
        }
        bool const free = spec.physics == EntitySpec::Physics::Free;
        if (spec.shape)
        {
            physics->addBox(FreeBox{ spec.id,
                                     spec.shape->size,
                                     spec.mass,
                                     boxInertia(spec.mass, spec.shape->size),
                                     record.nominal,
                                     spec.friction < 0.0 ? 1.0 : spec.friction });
        }
        else
        {
            (void)physics->load(spec.model, ChainOptions{ free, record.nominal, spec.friction });
        }
    }
    addRigidProps(*physics);
    m_robot->connect(std::move(physics));
}

bool Simulation::rigidProp(EntitySpec const& p_prop) const
{
    PhysicsSpec const& physics = m_scenario.physics;
    if (p_prop.type != EntitySpec::Type::Prop || !p_prop.shape)
    {
        return false;
    }
    if (physics.of(p_prop.id) == Fidelity::Rigid)
    {
        return true;
    }
    bool const box = p_prop.shape->type == ShapeSpec::Type::Box;
    bool rigid_gripper = false;
    bool rigid_box = false;
    for (EntitySpec const& entity : m_scenario.world.entities)
    {
        for (ActuatorSpec const& actuator : entity.actuators)
        {
            rigid_gripper = rigid_gripper || (actuator.type == ActuatorSpec::Type::Vacuum &&
                                              physics.of(actuator.name) == Fidelity::Rigid);
        }
        rigid_box = rigid_box || (entity.type == EntitySpec::Type::Prop && entity.shape &&
                                  entity.shape->type == ShapeSpec::Type::Box &&
                                  physics.of(entity.id) == Fidelity::Rigid);
    }
    // A box a rigid gripper may take; a container a rigid object may fall in.
    return box ? rigid_gripper : (rigid_gripper || rigid_box);
}

void Simulation::addRigidProps(MujocoBackend& p_physics)
{
    std::vector<RigidGrasp::Graspable> graspable;
    for (Record const& record : m_records)
    {
        EntitySpec const& spec = *record.spec;
        if (spec.type != EntitySpec::Type::Prop || !rigidProp(spec))
        {
            continue;
        }
        double const friction = spec.friction < 0.0 ? 1.0 : spec.friction;
        Vector3 const size = spec.shape->size;
        if (spec.shape->type == ShapeSpec::Type::Box)
        {
            p_physics.addBox(FreeBox{ spec.id, size, spec.mass, boxInertia(spec.mass, size),
                                      record.nominal, friction });
            graspable.push_back({ spec.id, 0.5 * size.z });
            continue;
        }
        // An open container: a floor and four walls, static, of the wall
        // thickness of the abstract level (scene::innerBounds).
        if (spec.randomize != decltype(spec.randomize){})
        {
            throw std::runtime_error("Prop '" + spec.id +
                                     "': a rigid container cannot be randomized (its walls are static)");
        }
        double const wall = ecs::CONTAINER_WALL_M;
        Pose const& at = record.nominal;
        auto add = [&](std::string const& p_name, Vector3 const& p_size, Vector3 const& p_offset)
        {
            FreeBox part{ p_name, p_size, 1.0, zero3(), at * Pose{ p_offset, Quaternion{} }, friction };
            part.fixed = true;
            p_physics.addBox(part);
        };
        add(spec.id + "_floor", Vector3(size.x, size.y, wall), Vector3(0.0, 0.0, -0.5 * size.z + 0.5 * wall));
        add(spec.id + "_wall_x0", Vector3(wall, size.y, size.z), Vector3(-0.5 * size.x + 0.5 * wall, 0.0, 0.0));
        add(spec.id + "_wall_x1", Vector3(wall, size.y, size.z), Vector3(0.5 * size.x - 0.5 * wall, 0.0, 0.0));
        add(spec.id + "_wall_y0", Vector3(size.x, wall, size.z), Vector3(0.0, -0.5 * size.y + 0.5 * wall, 0.0));
        add(spec.id + "_wall_y1", Vector3(size.x, wall, size.z), Vector3(0.0, 0.5 * size.y - 0.5 * wall, 0.0));
    }

    if (graspable.empty())
    {
        return;
    }
    // One weld per gripper and object; the object never collides with the
    // links of the tool that holds it.
    std::vector<std::string> tool_links;
    for (Record const& record : m_records)
    {
        if (record.spec->type == EntitySpec::Type::Tool)
        {
            tool_links.insert(tool_links.end(), record.links.begin(), record.links.end());
        }
    }
    for (EntitySpec const& entity : m_scenario.world.entities)
    {
        for (ActuatorSpec const& actuator : entity.actuators)
        {
            if (actuator.type != ActuatorSpec::Type::Vacuum)
            {
                continue;
            }
            std::string const link = actuator.link.empty() ? m_robot->tool() : actuator.link;
            for (RigidGrasp::Graspable const& object : graspable)
            {
                p_physics.addWeld(RigidGrasp::weldName(actuator.name, object.id), link, object.id);
                for (std::string const& tool_link : tool_links)
                {
                    if (tool_link != link)
                    {
                        p_physics.excludeContacts(tool_link, object.id);
                    }
                }
            }
        }
    }
    m_grasps.push_back(std::make_unique<RigidGrasp>(std::move(graspable)));
}

void Simulation::addSkills()
{
    ResourceManager const& resources = m_robot->resources();
    ActuatorSet const& actuators = m_robot->actuators();

    std::vector<ResourceRequirement> arm;
    if (auto const* group = actuators.first<JointGroup>())
    {
        arm.push_back(resources.require(group->name()));
    }
    std::vector<ResourceRequirement> everything;
    for (std::size_t i = 0; i < actuators.size(); ++i)
    {
        everything.push_back({ actuators.resource(i), Access::Exclusive });
    }
    auto describe = [](std::string p_name, std::vector<ResourceRequirement> p_resources)
    {
        SkillDescription description;
        description.name = std::move(p_name);
        description.resources = std::move(p_resources);
        description.priority = SKILL_PRIORITY;
        return description;
    };

    SkillScheduler& skills = *m_scheduler;
    if (!arm.empty())
    {
        skills.add<HomeSkill>(describe("Home", arm));
    }
    SkillDescription stop = describe("Stop", everything);
    stop.priority = STOP_PRIORITY;
    if (!everything.empty())
    {
        skills.add<StopSkill>(std::move(stop));
    }
}

void Simulation::loadTree()
{
    m_tree.reset();
    m_status = bt::Status::INVALID;
    std::filesystem::path const& file = m_scenario.controller.behavior_tree;
    if (file.empty())
    {
        return;
    }
    m_blackboard = std::make_shared<bt::Blackboard>();
    auto built = bt::Builder::fromFile(m_factory, file.string(), m_blackboard);
    if (!built)
    {
        throw std::runtime_error("Behavior tree '" + file.string() + "': " + built.getError());
    }
    m_tree = std::move(built.getValue());
    m_tree->clock(simulatedClock());
}

bt::Tree::ClockFunction Simulation::simulatedClock() const
{
    // Timeout, Delay, Wait and Cooldown read the simulated time: they follow
    // pause, speed and the step, and a run is reproducible.
    return [this]
    {
        std::chrono::duration<double> const elapsed(m_robot->time().value());
        return std::chrono::steady_clock::time_point(
            std::chrono::duration_cast<std::chrono::steady_clock::duration>(elapsed));
    };
}

void Simulation::reset(Seed p_seed)
{
    m_seed = p_seed;
    m_max_contacts = 0;
    m_prop_penetration = false;
    m_outcome = Outcome::Running;

    m_robot->resources().restoreAll();
    m_scheduler->reset();
    m_faults.reset(p_seed.derive("faults"));

    ActuatorSet const& actuators = m_robot->actuators();
    for (std::size_t i = 0; i < actuators.size(); ++i)
    {
        if (auto* vacuum = dynamic_cast<VacuumGripper*>(&actuators[i]))
        {
            vacuum->suction(false);
            vacuum->held({});
        }
    }

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

    m_robot->reset();

    // Props: the robot only knows the nominal layout (in its base frame); the
    // true positions are randomized.
    Random random(p_seed.derive("world"));
    Pose const to_base = m_robot->base().pose.inverse();
    m_world_model.clear();
    for (Record& record : m_records)
    {
        record.peak_contacts = 0;
        EntitySpec const& spec = *record.spec;
        if (spec.type != EntitySpec::Type::Prop)
        {
            continue;
        }
        Vector3 at = record.nominal.position;
        at.x += random.uniform(spec.randomize[0][0], spec.randomize[0][1]);
        at.y += random.uniform(spec.randomize[1][0], spec.randomize[1][1]);
        at.z += random.uniform(spec.randomize[2][0], spec.randomize[2][1]);
        // A rigid box is a free body: the backend moves it, syncBodies copies.
        RobotBackend* backend = m_robot->backend();
        if (record.body.empty() || backend == nullptr ||
            !backend->placeBody(record.body, Pose{ at, record.nominal.rotation }))
        {
            record.entity.position(toFloat(at)).rotation(toFloat(record.nominal.rotation));
        }
        m_world_model.add(spec.id, to_base * record.nominal.position, spec.shape->size);
    }
    syncBodies();
    m_world.update();
    for (Record& record : m_records)
    {
        record.initial = pose(record.spec->id);
    }

    loadTree();
    if (m_executor)
    {
        m_executor->reset(p_seed);
    }
    if (m_mission != nullptr)
    {
        m_mission->reset(*this, p_seed);
    }
    m_assertions = AssertionEngine(m_scenario.assertions);
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
    Pose const to_base = m_robot->base().pose.inverse();
    for (Record const& record : m_records)
    {
        if (record.spec->type == EntitySpec::Type::Prop)
        {
            m_world_model.place(record.spec->id, to_base * positionOf(record.entity), m_robot->time());
        }
    }
}

void Simulation::syncBodies()
{
    RobotBackend const* backend = m_robot->backend();
    compages::core::Quatf const& up = m_robot->urdfFrame();
    for (Record const& record : m_records)
    {
        if (record.body.empty() || !record.entity)
        {
            continue;
        }
        std::optional<Pose> const at =
            backend != nullptr ? backend->bodyPose(record.body) : std::nullopt;
        Pose const pose = at ? *at : record.nominal;
        compages::world::Entity entity = record.entity;
        if (record.spec->shape)
        {
            // Child of the Z-up frame: local pose is the world pose.
            entity.position(toFloat(pose.position)).rotation(toFloat(pose.rotation));
        }
        else
        {
            // Root of a loaded model, at the Compages root.
            entity.position(up * toFloat(pose.position)).rotation(up * toFloat(pose.rotation));
        }
    }
}

void Simulation::track()
{
    RobotBackend const* backend = m_robot->backend();
    if (backend == nullptr)
    {
        return;
    }
    m_max_contacts = std::max(m_max_contacts, backend->contacts());
    std::vector<Contact> const contacts = backend->contactList();
    for (Record& record : m_records)
    {
        if (!record.links.empty())
        {
            record.peak_contacts = std::max(record.peak_contacts, contactsOf(record, contacts));
        }
    }
}

void Simulation::step(Seconds p_dt)
{
    if (!m_suspended)
    {
        m_faults.update(m_robot->resources(), m_robot->time(), p_dt);
        m_context.time = m_robot->time();
        m_context.dt = p_dt;
        if (m_executor && !finished())
        {
            m_status = m_executor->tick(m_context);
        }
        else if (m_tree && !finished())
        {
            m_status = m_tree->tick();
        }
        m_scheduler->update(m_context);
    }

    // MuJoCo arm and rigid bodies, then what the grippers hold.
    m_robot->step(p_dt);
    for (std::unique_ptr<GraspModel> const& grasp : m_grasps)
    {
        grasp->update(*m_robot);
    }
    trackPropPenetration(*m_robot, m_prop_penetration);
    if (m_oracle)
    {
        observe();
    }
    track();
    syncBodies();

    if (m_suspended)
    {
        return;
    }
    if (m_mission != nullptr)
    {
        m_mission->step(*this, p_dt);
        if (m_tree == nullptr && m_executor == nullptr)
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
    settle();
}

void Simulation::settle()
{
    if (finished())
    {
        return;
    }
    m_assertions.update(time(),
                        [this](std::string_view p_condition) { return evaluate(p_condition); });

    if (m_status == bt::Status::SUCCESS)
    {
        finish(Outcome::ControllerSucceeded);
    }
    else if (m_status == bt::Status::FAILURE)
    {
        finish(Outcome::ControllerFailed);
    }
    else if (m_assertions.stopRequested())
    {
        finish(Outcome::AssertionFailed);
    }
    else if (!m_scenario.stop_when.empty() && evaluate(m_scenario.stop_when).holds)
    {
        finish(Outcome::StopCondition);
    }
    else if (m_scenario.timeout.value() > 0.0 && time() >= m_scenario.timeout)
    {
        finish(Outcome::Timeout);
    }
}

void Simulation::finish(Outcome p_outcome)
{
    m_outcome = p_outcome;
    m_assertions.conclude(time(),
                          [this](std::string_view p_condition) { return evaluate(p_condition); });
}

void Simulation::conclude()
{
    if (!finished())
    {
        finish(Outcome::Interrupted);
    }
}

} // namespace robotik
