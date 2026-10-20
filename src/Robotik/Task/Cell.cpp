// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "Robotik/Task/Cell.hpp"
#include "ShapeOf.hpp"

#include "Robotik/Robot/Backends/MujocoBackend.hpp"
#include "Robotik/Robot/Backends/PinocchioBackend.hpp"
#include "Robotik/Robot/Components/Objects.hpp"
#include "Robotik/Robot/Actuators.hpp"
#include "Robotik/Task/Grasp/ContainerBounds.hpp"
#include "Robotik/Robot/Sensors/ForceTorqueSensor.hpp"
#include "Robotik/Robot/Sensors/Imu.hpp"
#include "Robotik/Robot/Sensors/RangeScanner.hpp"
#include "Robotik/Task/Skills/MotionSkills.hpp"
#include "Robotik/Task/Skills/SkillNodes.hpp"
#include "Robotik/Task/Grasp/GraspModel.hpp"

#include "Compages/World/World.hpp"

#include <pugixml.hpp>

#include <algorithm>
#include <chrono>
#include <map>
#include <stdexcept>
#include <string>

#define SKILL_PRIORITY 100
#define STOP_PRIORITY 1000
#define WORLD_MODEL_GATE_M 0.10
/// Name of the Z-up frame entity the world entities hang from.
#define WORLD_FRAME_ENTITY "robotik.world"

namespace robotik
{

/// Reads an entity position as a double vector.
static Vector3 positionOf(compages::world::Entity p_entity)
{
    auto const at = p_entity.position();
    return { at.x, at.y, at.z };
}

/// Converts a double vector to the float vector the scene stores.
static compages::core::Vector3f toFloat(Vector3 const& p_v)
{
    return { static_cast<float>(p_v.x),
             static_cast<float>(p_v.y),
             static_cast<float>(p_v.z) };
}

/// Converts a double quaternion to the float quaternion the scene stores.
static compages::core::Quatf toFloat(Quaternion const& p_q)
{
    return { static_cast<float>(p_q.w),
             static_cast<float>(p_q.x),
             static_cast<float>(p_q.y),
             static_cast<float>(p_q.z) };
}

/// Links of a URDF and its root (the link no joint has as child).
struct UrdfLinks
{
    std::string root;
    std::unordered_set<std::string> names;
};

/// Reads each link name and its inertial origin from a URDF.
static UrdfLinks readLinks(std::filesystem::path const& p_urdf)
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

/// Solid box inertia at the centre (kg.m^2).
static Vector3 boxInertia(double p_mass, Vector3 const& p_size)
{
    double const x2 = p_size.x * p_size.x;
    double const y2 = p_size.y * p_size.y;
    double const z2 = p_size.z * p_size.z;
    return { p_mass * (y2 + z2) / 12.0,
             p_mass * (x2 + z2) / 12.0,
             p_mass * (x2 + y2) / 12.0 };
}

/// While the vacuum holds a prop, flag wall penetration (counts as a robot
/// collision). Sticky until @ref Cell::reset.
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
                p_entity.id() == held.id() ||
                p_entity.find<ecs::PhysicsBody>() != nullptr ||
                p_entity.find<ecs::Decoration>() != nullptr)
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

Cell::Cell(compages::world::World& p_world,
           CellDescription p_description,
                       SceneView* p_view,
                       Mission* p_mission)
    : m_world(p_world),
      m_description(std::move(p_description)),
      m_mission(p_mission),
      m_view(p_view),
      m_resources(std::make_shared<ResourceManager>()),
      m_robots(
          [&]()
          {
              WorldDescription const& world = m_description.world;
              std::vector<std::unique_ptr<RobotSession>> robots;
              for (EntitySpec const* spec : robotsOf(world))
              {
                  EntitySpec const* tool = mountedTool(world, spec->id);
                  robots.push_back(std::make_unique<RobotSession>(
                      p_world,
                      spec->model,
                      p_view,
                      tool != nullptr ? tool->model : std::filesystem::path{},
                      m_resources));
                  robots.back()->identify(spec->id);
              }
              return robots;
          }()),
      m_robot(m_robots.front().get()),
      m_scheduler(std::make_unique<SkillScheduler>(*m_resources)),
      m_faults(m_description.faults, m_description.random_faults)
{
    if (m_robots.size() > 1u)
    {
        for (std::unique_ptr<RobotSession> const& robot : m_robots)
        {
            m_team.push_back(robot.get());
        }
    }
    for (std::unique_ptr<RobotSession> const& robot : m_robots)
    {
        m_contexts.push_back(
            RobotContext{ *robot, m_world_model, {}, {}, m_team });
    }

    // Robot devices, props and bodies of the world file.
    build(p_view);

    // The mission adds its skills, detectors and dressing; it may also plug
    // its own backend.
    if (m_mission != nullptr)
    {
        m_mission->setup(*this, p_view);
    }
    bool const plugged =
        std::any_of(m_robots.begin(),
                    m_robots.end(),
                    [](std::unique_ptr<RobotSession> const& p_robot)
                    { return p_robot->backend() != nullptr; });
    if (!plugged)
    {
        connectBackend();
    }

    // Hold the home position, now with the backend.
    for (EntitySpec const* spec : robotsOf(m_description.world))
    {
        robot(spec->id)->hold(spec->home);
    }
    if (m_any_sensed)
    {
        m_world_model.gate(WORLD_MODEL_GATE_M);
    }

    addSkills();
    registerSkills(m_factory, *m_scheduler);
    if (m_description.controller.planner)
    {
        m_executor =
            std::make_unique<PlanExecutor>(*m_description.controller.planner,
                                           m_description.world,
                                           m_planning,
                                           m_description.id,
                                           m_factory,
                                           *m_scheduler);
        m_executor->clock(simulatedClock());
        m_executor->maxAge(m_description.perception.max_age);
    }
    reset();
}

Cell::~Cell() = default;

void Cell::build(SceneView* p_view)
{
    // Objects without physics; the rigid ones get their model with the backend.
    m_grasps.clear();
    m_grasps.push_back(std::make_unique<AbstractGrasp>());

    WorldDescription const& world = m_description.world;
    bool blind = true;
    for (EntitySpec const* robot_spec : robotsOf(world))
    {
        RobotSession& robot = *this->robot(robot_spec->id);
        EntitySpec const* tool = mountedTool(world, robot_spec->id);
        buildDevices(robot, *robot_spec, tool, p_view, blind);
    }

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

    // --- Where the beliefs about each prop come from. -----------------------
    m_oracle = false;
    // Beliefs a mission adds itself follow the default level.
    m_any_sensed = fidelityOf(m_description.perception, "", blind) == PerceptionFidelity::Sensed;
    for (Record& record : m_records)
    {
        if (record.spec->type != EntitySpec::Type::Prop)
        {
            continue;
        }
        record.perception = fidelityOf(m_description.perception, record.spec->id, blind);
        m_oracle = m_oracle || record.perception != PerceptionFidelity::Sensed;
        m_any_sensed = m_any_sensed || record.perception == PerceptionFidelity::Sensed;
    }
}

void Cell::buildDevices(RobotSession& p_robot,
                              EntitySpec const& p_spec,
                              EntitySpec const* p_tool,
                              SceneView* p_view,
                              bool& p_blind)
{
    p_robot.startPose(p_spec.pose);
    // Robot frames of the attachments are taken at home.
    p_robot.hold(p_spec.home);

    // --- Actuators: the robot's, then those of the mounted tool. ------------
    ActuatorSet& actuators = p_robot.actuators();
    std::vector<ActuatorSpec> specs = p_spec.actuators;
    if (p_tool != nullptr)
    {
        specs.insert(
            specs.end(), p_tool->actuators.begin(), p_tool->actuators.end());
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
                actuators.add<VacuumGripper>(
                    actuator.name, actuator.link, actuator.length);
                break;
        }
    }

    // --- Sensors: cameras get a render source in the simulator. -------------
    std::vector<SensorSpec> sensors = p_spec.sensors;
    if (p_tool != nullptr)
    {
        sensors.insert(
            sensors.end(), p_tool->sensors.begin(), p_tool->sensors.end());
    }
    auto requireFrame =
        [&p_robot](SensorSpec const& p_sensor, std::string const& p_frame)
    {
        if (!p_frame.empty() && !p_robot.link(p_frame))
        {
            throw std::runtime_error("Sensor '" + p_sensor.name +
                                     "': unknown link '" + p_frame + "'");
        }
    };
    for (SensorSpec const& spec : sensors)
    {
        switch (spec.type)
        {
            case SensorSpec::Type::Imu:
                requireFrame(spec, spec.imu.parent);
                p_robot.sensors().add<Imu>(spec.name, spec.imu);
                continue;
            case SensorSpec::Type::Range:
                requireFrame(spec, spec.range.parent);
                p_robot.sensors().add<RangeScanner>(spec.name, spec.range);
                continue;
            case SensorSpec::Type::ForceTorque:
                requireFrame(spec, spec.force_torque.link);
                p_robot.sensors().add<ForceTorqueSensor>(spec.name,
                                                         spec.force_torque);
                continue;
            case SensorSpec::Type::Camera:
                break;
        }
        requireFrame(spec, spec.camera.parent);
        Camera& camera = p_robot.sensors().add<Camera>(spec.name, spec.camera);
        compages::world::Entity link = spec.camera.parent.empty()
                                           ? p_robot.root()
                                           : p_robot.link(spec.camera.parent);
        if (p_view != nullptr)
        {
            camera.source(p_view->camera(camera, link));
        }
        p_blind = p_blind && camera.source() == nullptr;
        camera.onFrame(
            [this, &p_robot](CameraFrame const& p_frame)
            {
                Detections const& detections = m_perception.process(p_frame);
                if (m_robots.size() == 1u)
                {
                    m_world_model.update(detections);
                    return;
                }
                // Shared beliefs live in the world frame.
                Detections lifted = detections;
                lifted.camera = p_robot.base().pose * lifted.camera;
                m_world_model.update(lifted);
            });
    }
}

void Cell::buildRecord(Record& p_record, SceneView* p_view)
{
    EntitySpec const& spec = *p_record.spec;
    switch (spec.type)
    {
        case EntitySpec::Type::Robot:
        case EntitySpec::Type::Tool:
        {
            // Names of the physics backend: the robot prefix in a shared world.
            RobotSession& owner = ownerOf(spec);
            std::string const prefix = prefixOf(owner.id());
            for (std::string const& link : readLinks(spec.model).names)
            {
                p_record.links.insert(prefix + link);
            }
            if (spec.type == EntitySpec::Type::Robot)
            {
                p_record.entity = owner.root();
                p_record.nominal = spec.pose;
            }
            return;
        }
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
                    p_record.links = { spec.id + "_floor",
                                       spec.id + "_wall_x0",
                                       spec.id + "_wall_x1",
                                       spec.id + "_wall_y0",
                                       spec.id + "_wall_y1" };
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
        p_record.entity.parent(m_frame).set(object).set(
            ecs::PhysicsBody{ spec.id });
        if (p_view != nullptr)
        {
            p_view->object(p_record.entity, object);
        }
        return;
    }
    // MuJoCo names of a model body start with its id: two copies of a
    // model can share the world.
    UrdfLinks const links = readLinks(spec.model);
    std::string const prefix = spec.id + "/";
    p_record.body = prefix + links.root;
    for (std::string const& link : links.names)
    {
        p_record.links.insert(prefix + link);
    }
    p_record.frames = std::make_shared<PinocchioBackend>(spec.model);
    if (p_view != nullptr)
    {
        p_record.entity = p_view->model(m_world, spec.model);
    }
}

Pose Cell::resolveNominal(EntitySpec const& p_spec) const
{
    if (!p_spec.attach)
    {
        return p_spec.pose;
    }
    Attachment const& attach = *p_spec.attach;
    EntitySpec const* parent = findEntity(m_description.world, attach.parent);
    Pose base{ Vector3::ZERO, Quaternion{} };
    if (parent->type == EntitySpec::Type::Robot)
    {
        base = robot(parent->id)->worldPose(attach.parent_frame);
    }
    else
    {
        Record const* record = this->record(parent->id);
        if (record == nullptr)
        {
            throw std::runtime_error("Entity '" + p_spec.id + "' hangs from '" +
                                     parent->id + "', declared after it");
        }
        base = record->nominal;
        if (!attach.parent_frame.empty())
        {
            if (record->frames == nullptr ||
                !record->frames->hasFrame(attach.parent_frame))
            {
                throw std::runtime_error("Entity '" + p_spec.id + "': '" +
                                         parent->id + "' has no frame '" +
                                         attach.parent_frame + "'");
            }
            base = base * record->frames->framePose(attach.parent_frame);
        }
    }
    return base * attach.transform;
}

void Cell::connectBackend()
{
    WorldDescription const& world = m_description.world;
    MujocoOptions options = m_physics_options;
    // Rigid props rest on the ground: they need a floor.
    bool const rigid =
        std::any_of(world.entities.begin(),
                    world.entities.end(),
                    [this](EntitySpec const& p_entity)
                    {
                        return p_entity.type == EntitySpec::Type::Prop &&
                               rigidProp(p_entity);
                    });
    options.floor = options.floor || world.floor || rigid;
    options.floor_friction = world.floor_friction;
    auto physics = std::make_shared<MujocoBackend>(options);
    // The first robot is the host chain; the others stand in the world with
    // a prefix of their own, their tool with the same prefix.
    for (EntitySpec const* robot_spec : robotsOf(world))
    {
        std::string const prefix = prefixOf(robot_spec->id);
        ChainOptions chain;
        chain.pose = robot_spec->pose;
        chain.prefix = prefix;
        std::string const arm = physics->load(robot_spec->model, chain);
        if (EntitySpec const* tool = mountedTool(world, robot_spec->id))
        {
            ChainOptions mount;
            mount.prefix = prefix;
            std::string const mounted = physics->load(tool->model, mount);
            physics->attach(arm, "flange", mounted, "tool_mount");
        }
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
            physics->addBox(
                FreeBox{ spec.id,
                         spec.shape->size,
                         spec.mass,
                         boxInertia(spec.mass, spec.shape->size),
                         record.nominal,
                         spec.friction < 0.0 ? 1.0 : spec.friction });
        }
        else
        {
            ChainOptions chain{ free, record.nominal, spec.friction };
            chain.prefix = spec.id + "/";
            (void)physics->load(spec.model, chain);
        }
    }
    addRigidProps(*physics);
    // One physics for the cell; each robot reaches it through its names.
    for (std::unique_ptr<RobotSession> const& robot : m_robots)
    {
        robot->connect(MujocoBackend::member(physics, prefixOf(robot->id())));
    }
}

bool Cell::rigidProp(EntitySpec const& p_prop) const
{
    PhysicsSpec const& physics = m_description.physics;
    if (p_prop.type != EntitySpec::Type::Prop || !p_prop.shape)
    {
        return false;
    }
    if (fidelityOf(physics, p_prop.id) == Fidelity::Rigid)
    {
        return true;
    }
    bool const box = p_prop.shape->type == ShapeSpec::Type::Box;
    bool rigid_gripper = false;
    bool rigid_box = false;
    for (EntitySpec const& entity : m_description.world.entities)
    {
        for (ActuatorSpec const& actuator : entity.actuators)
        {
            rigid_gripper =
                rigid_gripper || (actuator.type == ActuatorSpec::Type::Vacuum &&
                                  fidelityOf(physics, actuator.name) == Fidelity::Rigid);
        }
        rigid_box = rigid_box ||
                    (entity.type == EntitySpec::Type::Prop && entity.shape &&
                     entity.shape->type == ShapeSpec::Type::Box &&
                     fidelityOf(physics, entity.id) == Fidelity::Rigid);
    }
    // A box a rigid gripper may take; a container a rigid object may fall in.
    return box ? rigid_gripper : (rigid_gripper || rigid_box);
}

void Cell::addRigidProps(MujocoBackend& p_physics)
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
            p_physics.addBox(FreeBox{ spec.id,
                                      size,
                                      spec.mass,
                                      boxInertia(spec.mass, size),
                                      record.nominal,
                                      friction });
            graspable.push_back({ spec.id, 0.5 * size.z });
            continue;
        }
        // An open container: a floor and four walls, static, of the wall
        // thickness of the abstract level (scene::innerBounds).
        if (spec.randomize != decltype(spec.randomize){})
        {
            throw std::runtime_error("Prop '" + spec.id +
                                     "': a rigid container cannot be "
                                     "randomized (its walls are static)");
        }
        double const wall = ecs::CONTAINER_WALL_M;
        Pose const& at = record.nominal;
        auto add = [&](std::string const& p_name,
                       Vector3 const& p_size,
                       Vector3 const& p_offset)
        {
            FreeBox part{ p_name,
                          p_size,
                          1.0,
                          Vector3::ZERO,
                          at * Pose{ p_offset, Quaternion{} },
                          friction };
            part.fixed = true;
            p_physics.addBox(part);
        };
        add(spec.id + "_floor",
            Vector3(size.x, size.y, wall),
            Vector3(0.0, 0.0, -0.5 * size.z + 0.5 * wall));
        add(spec.id + "_wall_x0",
            Vector3(wall, size.y, size.z),
            Vector3(-0.5 * size.x + 0.5 * wall, 0.0, 0.0));
        add(spec.id + "_wall_x1",
            Vector3(wall, size.y, size.z),
            Vector3(0.5 * size.x - 0.5 * wall, 0.0, 0.0));
        add(spec.id + "_wall_y0",
            Vector3(size.x, wall, size.z),
            Vector3(0.0, -0.5 * size.y + 0.5 * wall, 0.0));
        add(spec.id + "_wall_y1",
            Vector3(size.x, wall, size.z),
            Vector3(0.0, 0.5 * size.y - 0.5 * wall, 0.0));
    }

    if (graspable.empty())
    {
        return;
    }
    // One weld per gripper and object; the object never collides with the
    // links of the tool that holds it (the tool of another robot, a drill,
    // still touches it).
    for (EntitySpec const& entity : m_description.world.entities)
    {
        for (ActuatorSpec const& actuator : entity.actuators)
        {
            if (actuator.type != ActuatorSpec::Type::Vacuum)
            {
                continue;
            }
            RobotSession const& owner = ownerOf(entity);
            std::string const link =
                prefixOf(owner.id()) +
                (actuator.link.empty() ? owner.tool() : actuator.link);
            std::vector<std::string> tool_links;
            for (Record const& record : m_records)
            {
                if (record.spec->type == EntitySpec::Type::Tool &&
                    &ownerOf(*record.spec) == &owner)
                {
                    tool_links.insert(tool_links.end(),
                                      record.links.begin(),
                                      record.links.end());
                }
            }
            for (RigidGrasp::Graspable const& object : graspable)
            {
                p_physics.addWeld(
                    RigidGrasp::weldName(actuator.name, object.id),
                    link,
                    object.id);
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

void Cell::addSkills()
{
    auto describe =
        [](std::string p_name, std::vector<ResourceRequirement> p_resources)
    {
        SkillDescription description;
        description.name = std::move(p_name);
        description.resources = std::move(p_resources);
        description.priority = SKILL_PRIORITY;
        return description;
    };

    // Home and Stop of each robot; "left.Home" when the world has several.
    SkillScheduler& skills = *m_scheduler;
    for (std::unique_ptr<RobotSession> const& robot : m_robots)
    {
        ResourceManager const& resources = robot->resources();
        ActuatorSet const& actuators = robot->actuators();
        std::string const scope =
            m_robots.size() > 1u ? robot->id() + "." : std::string();

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
        if (!arm.empty())
        {
            SkillDescription home = describe(scope + "Home", arm);
            home.robot = robot->id();
            skills.add<HomeSkill>(std::move(home));
        }
        if (!everything.empty())
        {
            SkillDescription stop = describe(scope + "Stop", everything);
            stop.priority = STOP_PRIORITY;
            stop.robot = robot->id();
            skills.add<StopSkill>(std::move(stop));
        }
    }
}

Pose Cell::beliefFrame() const
{
    return m_robots.size() == 1u ? m_robot->base().pose
                                 : Pose{ Vector3::ZERO, Quaternion{} };
}

RobotSession* Cell::robot(std::string_view p_id) const
{
    for (std::unique_ptr<RobotSession> const& robot : m_robots)
    {
        if (robot->id() == p_id)
        {
            return robot.get();
        }
    }
    return nullptr;
}

RobotContext& Cell::context(std::string_view p_id)
{
    for (RobotContext& context : m_contexts)
    {
        if (context.robot.id() == p_id)
        {
            return context;
        }
    }
    throw std::invalid_argument("The world has no robot '" + std::string(p_id) +
                                "'");
}

RobotSession& Cell::ownerOf(EntitySpec const& p_spec) const
{
    std::string_view const id =
        p_spec.type == EntitySpec::Type::Robot ? std::string_view(p_spec.id)
        : p_spec.type == EntitySpec::Type::Tool && p_spec.attach
            ? std::string_view(p_spec.attach->parent)
            : std::string_view();
    RobotSession* const found = id.empty() ? nullptr : robot(id);
    return found != nullptr ? *found : *m_robot;
}

std::string Cell::prefixOf(std::string_view p_id) const
{
    return p_id == m_robot->id() ? std::string() : std::string(p_id) + "/";
}

void Cell::loadTree()
{
    m_tree.reset();
    m_status = bt::Status::INVALID;
    std::filesystem::path const& file = m_description.controller.behavior_tree;
    if (file.empty())
    {
        return;
    }
    m_blackboard = std::make_shared<bt::Blackboard>();
    auto built = bt::Builder::fromFile(m_factory, file.string(), m_blackboard);
    if (!built)
    {
        throw std::runtime_error("Behavior tree '" + file.string() +
                                 "': " + built.getError());
    }
    m_tree = std::move(built.getValue());
    m_tree->clock(simulatedClock());
}

bt::Tree::ClockFunction Cell::simulatedClock() const
{
    // Timeout, Delay, Wait and Cooldown read the simulated time: they follow
    // pause, speed and the step, and a run is reproducible.
    return [this]
    {
        std::chrono::duration<double> const elapsed(m_robot->time().value());
        return std::chrono::steady_clock::time_point(
            std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                elapsed));
    };
}

void Cell::reset(Seed p_seed)
{
    m_seed = p_seed;
    m_max_contacts = 0;
    m_prop_penetration.clear();
    m_controller_stopped = false;

    m_resources->restoreAll();
    m_scheduler->reset();
    m_faults.reset(p_seed.derive("faults"));
    m_perception_random = Random(p_seed.derive("perception"));

    for (std::unique_ptr<RobotSession> const& robot : m_robots)
    {
        ActuatorSet const& actuators = robot->actuators();
        for (std::size_t i = 0; i < actuators.size(); ++i)
        {
            if (auto* vacuum = dynamic_cast<VacuumGripper*>(&actuators[i]))
            {
                vacuum->suction(false);
                vacuum->held({});
            }
        }
    }

    for (std::unique_ptr<RobotSession> const& robot : m_robots)
    {
        SensorSet const& sensors = robot->sensors();
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
            else if (auto* force =
                         dynamic_cast<ForceTorqueSensor*>(&sensors[i]))
            {
                force->seed(p_seed.derive(force->name()));
            }
        }
    }

    for (std::unique_ptr<RobotSession> const& robot : m_robots)
    {
        robot->reset();
    }

    // Props: the robot only knows the nominal layout (in its base frame); the
    // true positions are randomized.
    Random random(p_seed.derive("world"));
    Pose const to_base = beliefFrame().inverse();
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
            !backend->placeBody(record.body,
                                Pose{ at, record.nominal.rotation }))
        {
            record.entity.position(toFloat(at))
                .rotation(toFloat(record.nominal.rotation));
        }
        m_world_model.add(
            spec.id, to_base * record.nominal.position, spec.shape->size);
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
}

Camera* Cell::camera() const
{
    for (std::unique_ptr<RobotSession> const& robot : m_robots)
    {
        if (Camera* camera = robot->sensors().first<Camera>())
        {
            return camera;
        }
    }
    return nullptr;
}

void Cell::observe()
{
    // No sensor at all, or one of some robot still works.
    bool any = false;
    bool sees = false;
    for (std::unique_ptr<RobotSession> const& robot : m_robots)
    {
        SensorSet const& sensors = robot->sensors();
        for (std::size_t i = 0; i < sensors.size(); ++i)
        {
            any = true;
            sees = sees || sensors.available(i);
        }
    }
    sees = sees || !any;
    if (!sees)
    {
        return;
    }
    Pose const to_base = beliefFrame().inverse();
    for (Record const& record : m_records)
    {
        if (record.spec->type == EntitySpec::Type::Prop)
        {
            m_world_model.place(record.spec->id,
                                to_base * positionOf(record.entity),
                                m_robot->time());
        }
    }
}

void Cell::perceiveTruth()
{
    // Same rule as the cameras: something must still be able to see.
    bool any = false;
    bool sees = false;
    for (std::unique_ptr<RobotSession> const& robot : m_robots)
    {
        SensorSet const& sensors = robot->sensors();
        for (std::size_t i = 0; i < sensors.size(); ++i)
        {
            any = true;
            sees = sees || sensors.available(i);
        }
    }
    if (any && !sees)
    {
        return;
    }
    Pose const to_base = beliefFrame().inverse();
    TruthNoise const& noise = m_description.perception.noise;
    for (Record const& record : m_records)
    {
        if (record.perception == PerceptionFidelity::Sensed ||
            record.spec->type != EntitySpec::Type::Prop)
        {
            continue;
        }
        Vector3 truth = positionOf(record.entity);
        if (record.perception == PerceptionFidelity::Truth)
        {
            m_world_model.place(record.spec->id, to_base * truth, m_robot->time(),
                                BeliefSource::Truth);
            continue;
        }
        // Noisy truth: in range of a robot, not missed, with noise.
        bool near = false;
        for (std::unique_ptr<RobotSession> const& robot : m_robots)
        {
            near = near || norm(truth - robot->base().pose.position) <= noise.range;
        }
        if (!near || m_perception_random.chance(noise.dropout))
        {
            continue;
        }
        truth.x = m_perception_random.normal(truth.x, noise.sigma);
        truth.y = m_perception_random.normal(truth.y, noise.sigma);
        truth.z = m_perception_random.normal(truth.z, noise.sigma);
        m_world_model.place(record.spec->id, to_base * truth, m_robot->time(),
                            BeliefSource::NoisyTruth);
    }
}

PerceptionFidelity Cell::perceptionOf(std::string_view p_id) const
{
    Record const* found = record(p_id);
    return found != nullptr ? found->perception : PerceptionFidelity::Sensed;
}

std::string Cell::perceptionSummary() const
{
    // The level of most props, then the exceptions: "truth, red_cube: sensed".
    std::map<PerceptionFidelity, int> count;
    for (Record const& record : m_records)
    {
        if (record.spec->type == EntitySpec::Type::Prop)
        {
            ++count[record.perception];
        }
    }
    if (count.empty())
    {
        return toString(m_description.perception.fidelity);
    }
    // Ties go to the default level of the description.
    PerceptionFidelity main = m_description.perception.fidelity;
    for (auto const& [level, n] : count)
    {
        main = n > count[main] ? level : main;
    }
    std::string text = toString(main);
    for (Record const& record : m_records)
    {
        if (record.spec->type == EntitySpec::Type::Prop && record.perception != main)
        {
            text += ", " + record.spec->id + ": " + toString(record.perception);
        }
    }
    return text;
}

void Cell::syncBodies()
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
            entity.position(toFloat(pose.position))
                .rotation(toFloat(pose.rotation));
        }
        else
        {
            // Root of a loaded model, at the Compages root.
            entity.position(up * toFloat(pose.position))
                .rotation(up * toFloat(pose.rotation));
        }
    }
}

void Cell::track()
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
            record.peak_contacts =
                std::max(record.peak_contacts, contactsOf(record, contacts));
        }
    }
}

void Cell::step(Seconds p_dt)
{
    if (!m_suspended)
    {
        m_faults.update(*m_resources, m_robot->time(), p_dt);
        for (RobotContext& context : m_contexts)
        {
            context.time = context.robot.time();
            context.dt = p_dt;
        }
        if (m_executor && !m_controller_stopped)
        {
            m_status = m_executor->tick(m_contexts.front());
        }
        else if (m_tree && !m_controller_stopped)
        {
            m_status = m_tree->tick();
        }
        m_scheduler->update(m_contexts);
    }

    // MuJoCo arms and rigid bodies (one step for all), then what the grippers
    // hold.
    for (std::unique_ptr<RobotSession> const& robot : m_robots)
    {
        robot->prepare(p_dt);
    }
    for (std::unique_ptr<RobotSession> const& robot : m_robots)
    {
        robot->advance(p_dt);
    }
    for (std::unique_ptr<RobotSession> const& robot : m_robots)
    {
        robot->sense();
    }
    for (std::unique_ptr<RobotSession> const& robot : m_robots)
    {
        for (std::unique_ptr<GraspModel> const& grasp : m_grasps)
        {
            grasp->update(*robot);
        }
        bool through = m_prop_penetration.contains(robot->id());
        trackPropPenetration(*robot, through);
        if (through)
        {
            m_prop_penetration.insert(robot->id());
        }
    }
    if (m_oracle)
    {
        perceiveTruth();
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
    if (m_status == bt::Status::SUCCESS || m_status == bt::Status::FAILURE)
    {
        m_controller_stopped = true; // the controller is done
    }
}

} // namespace robotik
