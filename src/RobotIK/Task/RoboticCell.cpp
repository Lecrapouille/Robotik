// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "RobotIK/Task/RoboticCell.hpp"
#include "ShapeOf.hpp"

#include "RobotIK/Robot/Backends/MujocoBackend.hpp"
#include "RobotIK/Robot/Kinematics.hpp"
#include "RobotIK/Robot/Components/Objects.hpp"
#include "RobotIK/Robot/Actuators.hpp"
#include "RobotIK/Task/Grasp/ContainerBounds.hpp"
#include "RobotIK/Robot/Sensors/ForceTorqueSensor.hpp"
#include "RobotIK/Robot/Sensors/Imu.hpp"
#include "RobotIK/Robot/Sensors/RangeScanner.hpp"
#include "RobotIK/Task/Skills/MotionSkills.hpp"
#include "RobotIK/Task/Skills/SkillNodes.hpp"
#include "RobotIK/Task/Grasp/GraspModel.hpp"
#include "RobotIK/Task/Story/Story.hpp"

#include "Compages/World/World.hpp"

#include <pugixml.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <stdexcept>
#include <string>
#include <utility>

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
    return { static_cast<double>(at.x),
             static_cast<double>(at.y),
             static_cast<double>(at.z) };
}

/// Converts a double vector to the float vector the scene stores.
static compages::Vector3f toFloat(Vector3 const& p_v)
{
    return { static_cast<float>(p_v.x),
             static_cast<float>(p_v.y),
             static_cast<float>(p_v.z) };
}

/// Converts a double quaternion to the float quaternion the scene stores.
static compages::Quatf toFloat(Quaternion const& p_q)
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
/// collision). Sticky until @ref RoboticCell::reset.
static void trackPropPenetration(RobotRuntime& p_robot,
                                 HeldObjects const& p_held,
                                 bool& p_flag)
{
    if (p_flag)
    {
        return;
    }
    VacuumGripper const* gripper =
        p_robot.robot().getActuators().first<VacuumGripper>();
    if (gripper == nullptr || !gripper->holding())
    {
        return;
    }
    compages::world::World& world = p_robot.getWorld();
    compages::world::EntityId const held_id = p_held.of(*gripper);
    if (!world.alive(held_id))
    {
        return;
    }
    compages::world::Entity const held = world.entity(held_id);
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

RoboticCell::RoboticCell(compages::world::World& p_world,
                         RoboticCellDescription p_description,
                         SceneView* p_view,
                         ScenarioLogic* p_logic)
    : m_world(p_world),
      m_description(std::move(p_description)),
      m_logic(p_logic),
      m_view(p_view),
      m_resources(std::make_shared<ResourceManager>()),
      m_robots(
          [&]()
          {
              // Before any robot: an empty world would leave no first one.
              robotik::require(validate(m_description));
              WorldDescription const& world = m_description.world;
              std::vector<std::unique_ptr<RobotRuntime>> robots;
              for (EntitySpec const* spec : robotsOf(world))
              {
                  robots.push_back(std::make_unique<RobotRuntime>(
                      p_world,
                      spec->model,
                      p_view,
                      toolMountOf(world, spec->id),
                      m_resources));
                  robots.back()->setId(spec->id);
                  robots.back()->setLocalization(m_description.localization);
              }
              return robots;
          }()),
      m_robot(m_robots.front().get()),
      m_scheduler(std::make_unique<SkillScheduler>(*m_resources)),
      m_faults(m_description.faults, m_description.random_faults),
      m_factory(std::make_unique<bt::NodeFactory>())
{
    m_queries = builtinQueries();
    for (std::unique_ptr<RobotRuntime> const& robot : m_robots)
    {
        m_runtime_views.push_back(robot.get());
        m_robot_views.push_back(&robot->robot());
    }
    if (m_robots.size() > 1U)
    {
        m_team = m_robot_views;
    }
    for (std::unique_ptr<RobotRuntime> const& robot : m_robots)
    {
        m_contexts.push_back(
            RobotContext{ robot->robot(), m_world_model, {}, {}, m_team });
    }

    // Robot devices, props and bodies of the world file.
    build(p_view);

    // The scenario logic adds its skills, detectors and dressing; it may also plug
    // its own backend.
    if (m_logic != nullptr)
    {
        m_logic->setup(*this, p_view);
    }
    bool const plugged =
        std::ranges::any_of(m_robots,
                            [](std::unique_ptr<RobotRuntime> const& p_robot)
                            { return p_robot->getBackend() != nullptr; });
    if (!plugged)
    {
        connectBackend();
    }

    // Hold the home position, now with the backend.
    for (EntitySpec const* spec : robotsOf(m_description.world))
    {
        runtime(spec->id)->hold(spec->home);
    }
    if (m_any_sensed)
    {
        m_world_model.gate(WORLD_MODEL_GATE_M);
    }

    addSkills();
    registerSkills(*m_factory, *m_scheduler);
    if (m_description.controller.planner)
    {
        achieve(*m_description.controller.planner);
    }
    // After the scenario logic: the story may name its metrics and queries.
    bool const actors = std::ranges::any_of(
        m_description.world.entities,
        [](EntitySpec const& p_entity)
        { return p_entity.type == EntitySpec::Type::Actor; });
    if (actors || !m_description.story.events.empty())
    {
        m_story = std::make_unique<Story>(m_description.story,
                                          m_description.world,
                                          [this](std::string_view p_text)
                                          { return compile(p_text); });
    }
    reset();
}

void RoboticCell::achieve(PlannerSpec p_spec)
{
    if (!m_description.controller.behavior_tree.empty())
    {
        throw std::runtime_error("achieve: the controller is a behavior tree; "
                                 "give a tree or a goal, "
                                 "not both");
    }
    auto executor = std::make_unique<PlanExecutor>(p_spec,
                                                   m_description.world,
                                                   m_planning,
                                                   m_description.id,
                                                   *m_factory,
                                                   *m_scheduler);
    executor->clock(simulatedClock());
    executor->maxAge(m_description.perception.max_age);
    executor->reset(m_seed);
    m_executor = std::move(executor);
    m_description.controller.planner = std::move(p_spec);
    m_status = Status::IDLE;
    m_controller_stopped = false;
}

/// A run has a limit >= 0 and steps of dt > 0, both finite: otherwise the
/// clock never reaches the end and the loop never stops.
static void checkRun(Seconds p_limit, Seconds p_dt)
{
    if (!(p_limit.value() >= 0.0) || !std::isfinite(p_limit.value()) ||
        !(p_dt.value() > 0.0) || !std::isfinite(p_dt.value()))
    {
        throw std::invalid_argument(
            "RoboticCell::run: the limit must be >= 0 and dt > 0, both finite");
    }
}

SkillState RoboticCell::run(SkillId p_skill, Seconds p_limit, Seconds p_dt)
{
    checkRun(p_limit, p_dt);
    m_scheduler->request(p_skill);
    Seconds const end = time() + p_limit;
    for (;;)
    {
        SkillState const state = m_scheduler->state(p_skill);
        bool const over = state == SkillState::Succeeded ||
                          state == SkillState::Failed ||
                          state == SkillState::Cancelled;
        if (over || time() >= end)
        {
            return state;
        }
        step(p_dt);
    }
}

Status RoboticCell::run(Seconds p_limit, Seconds p_dt)
{
    checkRun(p_limit, p_dt);
    Seconds const end = time() + p_limit;
    while (m_status != Status::SUCCESS && m_status != Status::FAILURE &&
           time() < end)
    {
        step(p_dt);
    }
    return m_status;
}

RoboticCell::~RoboticCell() = default;

void RoboticCell::build(SceneView* p_view)
{
    // Objects without physics; the rigid ones get their model with the backend.
    m_grasps.clear();
    m_grasps.push_back(std::make_unique<AbstractGrasp>());

    WorldDescription const& world = m_description.world;
    bool blind = true;
    for (EntitySpec const* robot_spec : robotsOf(world))
    {
        RobotRuntime& robot = *runtime(robot_spec->id);
        EntitySpec const* tool = mountedTool(world, robot_spec->id);
        buildDevices(robot, *robot_spec, tool, p_view, blind);
    }

    // --- Entities: one record each, in file order. ---------------------------
    m_frame = m_world.entity(WORLD_FRAME_ENTITY);
    m_frame.rotation(m_robot->getUrdfFrame());
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
    // Beliefs a scenario logic adds itself follow the default level.
    m_any_sensed = fidelityOf(m_description.perception, "", blind) ==
                   PerceptionFidelity::Sensed;
    for (Record& record : m_records)
    {
        if (record.spec->type != EntitySpec::Type::Prop)
        {
            continue;
        }
        record.perception =
            fidelityOf(m_description.perception, record.spec->id, blind);
        m_oracle = m_oracle || record.perception != PerceptionFidelity::Sensed;
        m_any_sensed =
            m_any_sensed || record.perception == PerceptionFidelity::Sensed;
    }
}

void RoboticCell::buildDevices(RobotRuntime& p_robot,
                               EntitySpec const& p_spec,
                               EntitySpec const* p_tool,
                               SceneView* p_view,
                               bool& p_blind)
{
    p_robot.setStartPose(p_spec.pose);
    // Robot frames of the attachments are taken at home.
    p_robot.hold(p_spec.home);

    // --- Actuators: the robot's, then those of the mounted tool. ------------
    ActuatorSet& actuators = p_robot.robot().getActuators();
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
                p_robot.addActuator<JointGroup>(actuator.name, actuator.joints);
                break;
            case ActuatorSpec::Type::Motor:
                p_robot.addActuator<Motor>(actuator.name,
                                           actuator.joints.front());
                break;
            case ActuatorSpec::Type::Vacuum:
                p_robot.addActuator<VacuumGripper>(
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
        if (!p_frame.empty() && !p_robot.getLink(p_frame))
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
                p_robot.addSensor<Imu>(spec.name, spec.imu);
                continue;
            case SensorSpec::Type::Range:
                requireFrame(spec, spec.range.parent);
                p_robot.addSensor<RangeScanner>(spec.name, spec.range);
                continue;
            case SensorSpec::Type::ForceTorque:
                requireFrame(spec, spec.force_torque.link);
                p_robot.addSensor<ForceTorqueSensor>(spec.name,
                                                            spec.force_torque);
                continue;
            case SensorSpec::Type::Camera:
                break;
        }
        requireFrame(spec, spec.camera.parent);
        Camera& camera =
            p_robot.addSensor<Camera>(spec.name, spec.camera);
        compages::world::Entity link =
            spec.camera.parent.empty() ? p_robot.getRoot()
                                       : p_robot.getLink(spec.camera.parent);
        if (p_view != nullptr)
        {
            camera.source(p_view->camera(camera, link));
        }
        p_blind = p_blind && camera.source() == nullptr;
        camera.onFrame(
            [this, &p_robot](CameraFrame const& p_frame)
            {
                Detections const& detections = m_perception.process(p_frame);
                if (m_robots.size() == 1U)
                {
                    m_world_model.update(detections);
                    return;
                }
                // Shared beliefs live in the world frame.
                Detections lifted = detections;
                lifted.camera = p_robot.robot().getBase().pose * lifted.camera;
                m_world_model.update(lifted);
            });
    }
}

void RoboticCell::buildRecord(Record& p_record, SceneView* p_view)
{
    EntitySpec const& spec = *p_record.spec;
    switch (spec.type)
    {
        case EntitySpec::Type::Robot:
        case EntitySpec::Type::Tool:
        {
            // Names of the physics backend: the robot prefix in a shared world.
            RobotRuntime const& owner = ownerOf(spec);
            std::string const prefix = prefixOf(owner.robot().getId());
            for (std::string const& link : readLinks(spec.model).names)
            {
                p_record.links.insert(prefix + link);
            }
            if (spec.type == EntitySpec::Type::Robot)
            {
                p_record.entity = owner.getRoot();
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
        case EntitySpec::Type::Actor:
        {
            // Kinematic: the story moves it. A box stands in when the view
            // has no model to draw, or headless.
            p_record.entity = m_world.entity(spec.id);
            p_record.entity.parent(m_frame);
            if (spec.shape)
            {
                ecs::Shape const object = shapeOf(spec);
                p_record.entity.set(object);
                if (p_view != nullptr && spec.model.empty())
                {
                    p_view->object(p_record.entity, object);
                }
            }
            if (p_view != nullptr && !spec.model.empty())
            {
                p_view->actor(p_record.entity, spec.model, spec.actor.model_yaw,
                              spec.actor.model_scale);
            }
            p_record.nominal = spec.pose;
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
    p_record.frames = std::make_shared<Kinematics>(spec.model);
    if (p_view != nullptr)
    {
        p_record.entity = p_view->model(m_world, spec.model);
    }
}

Pose RoboticCell::resolveNominal(EntitySpec const& p_spec) const
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
        base = robot(parent->id)->requireWorldPose(attach.parent_frame);
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
            base = base * record->frames->requireFramePose(attach.parent_frame);
        }
    }
    return base * attach.transform;
}

void RoboticCell::connectBackend()
{
    WorldDescription const& world = m_description.world;
    MujocoOptions options = m_physics_options;
    // Rigid props rest on the ground: they need a floor.
    bool const rigid = std::ranges::any_of(
        world.entities,
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
        (void)physics->loadRobot(
            robot_spec->model, toolMountOf(world, robot_spec->id), chain);
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
            ChainOptions chain{ free, record.nominal, spec.friction,
                                spec.id + "/" };
            (void)physics->load(spec.model, chain);
        }
    }
    addRigidProps(*physics);
    // One physics for the cell; each robot reaches it through its names.
    for (std::unique_ptr<RobotRuntime> const& robot : m_robots)
    {
        robot->connect(
            MujocoBackend::member(physics, prefixOf(robot->robot().getId())));
    }
}

bool RoboticCell::rigidProp(EntitySpec const& p_prop) const
{
    PhysicsSpec const& physics = m_description.physics;
    if (p_prop.type != EntitySpec::Type::Prop || !p_prop.shape)
    {
        return false;
    }
    if (fidelityOf(physics, p_prop.id) == PhysicsFidelity::Rigid)
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
                rigid_gripper ||
                (actuator.type == ActuatorSpec::Type::Vacuum &&
                 fidelityOf(physics, actuator.name) == PhysicsFidelity::Rigid);
        }
        rigid_box = rigid_box ||
                    (entity.type == EntitySpec::Type::Prop && entity.shape &&
                     entity.shape->type == ShapeSpec::Type::Box &&
                     fidelityOf(physics, entity.id) == PhysicsFidelity::Rigid);
    }
    // A box a rigid gripper may take; a container a rigid object may fall in.
    return box ? rigid_gripper : (rigid_gripper || rigid_box);
}

void RoboticCell::addRigidProps(MujocoBackend& p_physics)
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
            RobotRuntime const& owner = ownerOf(entity);
            std::string const link =
                prefixOf(owner.robot().getId()) + (actuator.link.empty()
                                                       ? owner.robot().getTool()
                                                       : actuator.link);
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

void RoboticCell::addSkills()
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
    for (std::unique_ptr<RobotRuntime> const& robot : m_robots)
    {
        ResourceManager const& resources = robot->getResources();
        ActuatorSet const& actuators = robot->robot().getActuators();
        std::string const scope =
            m_robots.size() > 1U ? robot->robot().getId() + "." : std::string();

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
            home.robot = robot->robot().getId();
            skills.add<HomeSkill>(std::move(home));
        }
        if (!everything.empty())
        {
            SkillDescription stop = describe(scope + "Stop", everything);
            stop.priority = STOP_PRIORITY;
            stop.urgent = true;
            stop.robot = robot->robot().getId();
            skills.add<StopSkill>(std::move(stop));
        }
    }
}

Pose RoboticCell::beliefFrame() const
{
    return m_robots.size() == 1U ? m_robot->robot().getBase().pose
                                 : Pose{ Vector3::ZERO, Quaternion{} };
}

Pose RoboticCell::trueBeliefFrame() const
{
    return m_robots.size() == 1U ? m_robot->getTrueBase().pose
                                 : Pose{ Vector3::ZERO, Quaternion{} };
}

RobotRuntime const* RoboticCell::runtime(std::string_view p_id) const
{
    return const_cast<RoboticCell*>(this)->runtime(p_id);
}

RobotRuntime* RoboticCell::runtime(std::string_view p_id)
{
    for (std::unique_ptr<RobotRuntime> const& runtime : m_robots)
    {
        if (runtime->robot().getId() == p_id)
        {
            return runtime.get();
        }
    }
    return nullptr;
}

Robot const* RoboticCell::robot(std::string_view p_id) const
{
    RobotRuntime const* found = runtime(p_id);
    return found != nullptr ? &found->robot() : nullptr;
}

Robot* RoboticCell::robot(std::string_view p_id)
{
    RobotRuntime* found = runtime(p_id);
    return found != nullptr ? &found->robot() : nullptr;
}

RobotContext& RoboticCell::requireContext(std::string_view p_id)
{
    for (RobotContext& context : m_contexts)
    {
        if (context.robot.getId() == p_id)
        {
            return context;
        }
    }
    throw std::invalid_argument("The world has no robot '" + std::string(p_id) +
                                "'");
}

RobotRuntime const& RoboticCell::ownerOf(EntitySpec const& p_spec) const
{
    std::string_view const id =
        p_spec.type == EntitySpec::Type::Robot ? std::string_view(p_spec.id)
        : p_spec.type == EntitySpec::Type::Tool && p_spec.attach
            ? std::string_view(p_spec.attach->parent)
            : std::string_view();
    RobotRuntime const* const found = id.empty() ? nullptr : runtime(id);
    return found != nullptr ? *found : *m_robot;
}

std::string RoboticCell::prefixOf(std::string_view p_id) const
{
    return p_id == m_robot->robot().getId() ? std::string()
                                            : std::string(p_id) + "/";
}

void RoboticCell::loadTree()
{
    m_tree.reset();
    m_status = Status::IDLE;
    std::filesystem::path const& file = m_description.controller.behavior_tree;
    if (file.empty())
    {
        return;
    }
    m_blackboard = std::make_shared<bt::Blackboard>();
    auto built = bt::Builder::fromFile(*m_factory, file.string(), m_blackboard);
    if (!built)
    {
        throw std::runtime_error("Behavior tree '" + file.string() +
                                 "': " + built.getError());
    }
    m_tree = std::move(built.getValue());
    m_tree->clock(simulatedClock());
}

bt::NodeFactory& RoboticCell::factory()
{
    return *m_factory;
}

bt::Tree const* RoboticCell::tree() const
{
    return m_executor ? m_executor->tree() : m_tree.get();
}

TreeClock RoboticCell::simulatedClock() const
{
    // Timeout, Delay, Wait and Cooldown read the simulated time: they follow
    // pause, speed and the step, and a run is reproducible.
    return [this]
    {
        std::chrono::duration<double> const elapsed(
            m_robot->getTime().value());
        return std::chrono::steady_clock::time_point(
            std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                elapsed));
    };
}

void RoboticCell::reset(Seed p_seed)
{
    m_seed = p_seed;
    m_max_contacts = 0;
    m_prop_penetration.clear();
    m_controller_stopped = false;
    m_ticked = false;
    m_last_tick = Seconds{};
    m_stalled_steps = 0U;

    m_resources->restoreAll();
    m_scheduler->reset();
    m_faults.reset(p_seed.derive("faults"));
    m_perception_random = Random(p_seed.derive("perception"));

    for (std::unique_ptr<RobotRuntime> const& robot : m_robots)
    {
        ActuatorSet& actuators = robot->robot().getActuators();
        for (std::size_t i = 0; i < actuators.size(); ++i)
        {
            if (auto* vacuum = dynamic_cast<VacuumGripper*>(&actuators[i]))
            {
                vacuum->suction(false);
                m_held.drop(*robot, *vacuum);
            }
        }
    }

    for (std::unique_ptr<RobotRuntime> const& robot : m_robots)
    {
        SensorSet& sensors = robot->getSensors();
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

    for (std::unique_ptr<RobotRuntime> const& robot : m_robots)
    {
        robot->reset();
    }

    // Props: the robot only knows the nominal layout (in its base frame); the
    // true positions are randomized.
    Random random(p_seed.derive("world"));
    Pose const to_base = trueBeliefFrame().inverse();
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
        PhysicsWorld* physics = m_robot->getPhysics();
        if (record.body.empty() || physics == nullptr ||
            !physics->placeBody(record.body,
                                Pose{ at, record.nominal.rotation }))
        {
            record.entity.position(toFloat(at))
                .rotation(toFloat(record.nominal.rotation));
        }
        m_world_model.add(
            spec.id, to_base * record.nominal.position, spec.shape->size);
    }
    // Actors: where the world file puts them, shifted per seed like props.
    for (Record& record : m_records)
    {
        EntitySpec const& spec = *record.spec;
        if (spec.type != EntitySpec::Type::Actor)
        {
            continue;
        }
        Vector3 at = record.nominal.position;
        at.x += random.uniform(spec.randomize[0][0], spec.randomize[0][1]);
        at.y += random.uniform(spec.randomize[1][0], spec.randomize[1][1]);
        at.z += random.uniform(spec.randomize[2][0], spec.randomize[2][1]);
        record.entity.position(toFloat(at))
            .rotation(toFloat(record.nominal.rotation));
    }
    syncBodies();
    m_world.update();
    for (Record& record : m_records)
    {
        record.initial = requirePose(record.spec->id);
    }
    if (m_story)
    {
        m_story->reset(*this, m_view, p_seed);
    }

    loadTree();
    if (m_executor)
    {
        m_executor->reset(p_seed);
    }
    if (m_logic != nullptr)
    {
        m_logic->reset(*this, p_seed);
    }
}

Camera const* RoboticCell::camera() const
{
    return const_cast<RoboticCell*>(this)->camera();
}

Camera* RoboticCell::camera()
{
    for (std::unique_ptr<RobotRuntime> const& robot : m_robots)
    {
        if (Camera* camera = robot->getSensors().first<Camera>())
        {
            return camera;
        }
    }
    return nullptr;
}

void RoboticCell::observe()
{
    // No sensor at all, or one of some robot still works.
    bool any = false;
    bool sees = false;
    for (std::unique_ptr<RobotRuntime> const& robot : m_robots)
    {
        SensorSet const& sensors = robot->getSensors();
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
    Pose const to_base = trueBeliefFrame().inverse();
    for (Record const& record : m_records)
    {
        if (record.spec->type == EntitySpec::Type::Prop)
        {
            m_world_model.place(record.spec->id,
                                to_base * positionOf(record.entity),
                                m_robot->getTime());
        }
    }
}

void RoboticCell::perceiveTruth()
{
    // Same rule as the cameras: something must still be able to see.
    bool any = false;
    bool sees = false;
    for (std::unique_ptr<RobotRuntime> const& robot : m_robots)
    {
        SensorSet const& sensors = robot->getSensors();
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
    Pose const to_base = trueBeliefFrame().inverse();
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
            m_world_model.place(record.spec->id,
                                to_base * truth,
                                m_robot->getTime(),
                                BeliefSource::Truth);
            continue;
        }
        // Noisy truth: in range of a robot, not missed, with noise.
        bool near = false;
        for (std::unique_ptr<RobotRuntime> const& robot : m_robots)
        {
            near = near || norm(truth - robot->getTrueBase().pose.position) <=
                               noise.range;
        }
        if (!near || m_perception_random.chance(noise.dropout))
        {
            continue;
        }
        truth.x = m_perception_random.normal(truth.x, noise.sigma);
        truth.y = m_perception_random.normal(truth.y, noise.sigma);
        truth.z = m_perception_random.normal(truth.z, noise.sigma);
        m_world_model.place(record.spec->id,
                            to_base * truth,
                            m_robot->getTime(),
                            BeliefSource::NoisyTruth);
    }
}

PerceptionFidelity RoboticCell::perceptionOf(std::string_view p_id) const
{
    Record const* found = record(p_id);
    return found != nullptr ? found->perception : PerceptionFidelity::Sensed;
}

std::string RoboticCell::perceptionSummary() const
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
        if (record.spec->type == EntitySpec::Type::Prop &&
            record.perception != main)
        {
            text += ", " + record.spec->id + ": " + toString(record.perception);
        }
    }
    return text;
}

void RoboticCell::syncBodies()
{
    PhysicsWorld const* physics = m_robot->getPhysics();
    compages::Quatf const& up = m_robot->getUrdfFrame();
    for (Record const& record : m_records)
    {
        if (record.body.empty() || !record.entity)
        {
            continue;
        }
        std::optional<Pose> const at =
            physics != nullptr ? physics->bodyPose(record.body) : std::nullopt;
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

void RoboticCell::track()
{
    PhysicsWorld const* physics = m_robot->getPhysics();
    if (physics == nullptr)
    {
        return;
    }
    m_max_contacts = std::max(m_max_contacts, physics->contacts());
    std::vector<Contact> const contacts = physics->contactList();
    for (Record& record : m_records)
    {
        if (!record.links.empty())
        {
            record.peak_contacts =
                std::max(record.peak_contacts, contactsOf(record, contacts));
        }
    }
}

void RoboticCell::step(Seconds p_dt)
{
    if (!(p_dt.value() > 0.0) || !std::isfinite(p_dt.value()))
    {
        throw std::invalid_argument("RoboticCell::step: dt must be positive "
                                    "and finite");
    }
    // The clock comes from the robot (its backend). The decision side gets
    // the time measured since its last tick: the asked dt in a simulation,
    // the true one on hardware. A clock that did not move skips the tick.
    Seconds const before = m_robot->getTime();
    Seconds const elapsed = m_ticked ? before - m_last_tick : p_dt;
    bool const stalled = !(elapsed.value() > 0.0);
    if (stalled)
    {
        ++m_stalled_steps;
    }
    else
    {
        m_ticked = true;
        m_last_tick = before;
    }
    if (!m_suspended && !stalled)
    {
        m_faults.update(*m_resources, before, elapsed);
        // One context per robot, in the same order (see the constructor).
        for (std::size_t i = 0; i < m_contexts.size(); ++i)
        {
            m_contexts[i].time = m_robots[i]->getTime();
            m_contexts[i].dt = elapsed;
        }
        if (m_executor && !m_controller_stopped)
        {
            m_status = m_executor->tick(m_contexts.front());
        }
        else if (m_tree && !m_controller_stopped)
        {
            m_status = toStatus(m_tree->tick());
        }
        m_scheduler->update(m_contexts);
    }

    // MuJoCo arms and rigid bodies (one step for all), then what the grippers
    // hold.
    for (std::unique_ptr<RobotRuntime> const& robot : m_robots)
    {
        robot->prepare(p_dt);
    }
    for (std::unique_ptr<RobotRuntime> const& robot : m_robots)
    {
        robot->advance(p_dt);
    }
    // One time for all: the entities take the bodies of this physics step
    // before the sensors, the grippers and the truth beliefs read them.
    syncBodies();
    for (std::unique_ptr<RobotRuntime> const& robot : m_robots)
    {
        robot->sense();
    }
    for (std::unique_ptr<RobotRuntime> const& robot : m_robots)
    {
        for (std::unique_ptr<GraspModel> const& grasp : m_grasps)
        {
            grasp->update(*robot, m_held);
        }
        bool through = m_prop_penetration.contains(robot->robot().getId());
        trackPropPenetration(*robot, m_held, through);
        if (through)
        {
            m_prop_penetration.insert(robot->robot().getId());
        }
    }
    if (m_oracle)
    {
        perceiveTruth();
    }
    track();

    if (m_suspended)
    {
        return;
    }
    // The actors are part of what the world does by itself.
    if (m_story)
    {
        m_story->step(*this, m_view, p_dt);
    }
    if (m_logic != nullptr)
    {
        // The robot side sees the time after the physics step, like status,
        // and the duration of that step as the clock measured it.
        for (std::size_t i = 0; i < m_contexts.size(); ++i)
        {
            m_contexts[i].time = m_robots[i]->getTime();
            m_contexts[i].dt = m_robot->getTime() - before;
        }
        m_logic->act(m_contexts.front(), p_dt);
        m_logic->simulate(*this, p_dt);
        m_logic->observe(std::as_const(*this), p_dt);
        if (m_tree == nullptr && m_executor == nullptr)
        {
            switch (m_logic->status(m_contexts.front()))
            {
                case Status::SUCCESS:
                    m_status = Status::SUCCESS;
                    break;
                case Status::FAILURE:
                    m_status = Status::FAILURE;
                    break;
                default:
                    break;
            }
        }
    }
    if (m_status == Status::SUCCESS || m_status == Status::FAILURE)
    {
        m_controller_stopped = true; // the controller is done
    }
}

} // namespace robotik
