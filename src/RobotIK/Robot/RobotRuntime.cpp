// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "RobotIK/Robot/RobotRuntime.hpp"

#include "RobotIK/Robot/Components/Robots.hpp"
#include "RobotIK/Robot/Kinematics.hpp"

#include "Compages/World/Import/UrdfLoader.hpp"
#include "Compages/World/World.hpp"

#include <pugixml.hpp>

#include <cmath>
#include <stdexcept>

namespace robotik
{

/// A joint read from a URDF: name, type and child link.
struct UrdfJoint
{
    std::string name;
    std::string type;
    std::string child;
};

/// Robot name and moving joints read from a URDF.
struct UrdfRobot
{
    std::string name;
    std::vector<UrdfJoint> joints;
};

/// Reads the robot name and its joints from a URDF.
static Result<UrdfRobot> parseUrdf(std::filesystem::path const& p_urdf)
{
    pugi::xml_document document;
    pugi::xml_parse_result const parsed =
        document.load_file(p_urdf.string().c_str());
    if (!parsed)
    {
        return failure("Failed to parse URDF '" + p_urdf.string() +
                       "': " + parsed.description());
    }
    pugi::xml_node const robot = document.child("robot");
    UrdfRobot result{ robot.attribute("name").as_string(), {} };
    for (pugi::xml_node node : robot.children("joint"))
    {
        result.joints.push_back(
            { node.attribute("name").as_string(),
              node.attribute("type").as_string(),
              node.child("child").attribute("link").as_string() });
    }
    return result;
}

/// Same as @ref parseUrdf, for the constructor: a file that cannot be read
/// throws.
static UrdfRobot readUrdf(std::filesystem::path const& p_urdf)
{
    Result<UrdfRobot> parsed = parseUrdf(p_urdf);
    if (!parsed)
    {
        throw std::runtime_error(parsed.error());
    }
    return std::move(parsed.value());
}

/// True for a revolute, continuous or prismatic joint.
static bool moving(std::string_view p_type)
{
    return p_type == "revolute" || p_type == "continuous" ||
           p_type == "prismatic";
}

Result<std::unordered_map<std::string, JointType>>
readJointTypes(std::filesystem::path const& p_urdf)
{
    using Types = std::unordered_map<std::string, JointType>;
    UrdfRobot urdf;
    ROBOTIK_TRY_ASSIGN(urdf, parseUrdf(p_urdf));
    Types types;
    for (UrdfJoint const& joint : urdf.joints)
    {
        if (joint.type == "revolute")
        {
            types.emplace(joint.name, JointType::Revolute);
        }
        else if (joint.type == "continuous")
        {
            types.emplace(joint.name, JointType::Continuous);
        }
        else if (joint.type == "prismatic")
        {
            types.emplace(joint.name, JointType::Prismatic);
        }
    }
    return types;
}

/// Depth-first search of an entity by name.
static compages::world::Entity findNamed(compages::world::Entity p_root,
                                         std::string_view p_name)
{
    if (!p_root || p_root.name() == p_name)
    {
        return p_root;
    }
    compages::world::Entity found;
    p_root.children(
        [&found, p_name](compages::world::Entity p_child)
        {
            if (!found)
            {
                found = findNamed(p_child, p_name);
            }
        });
    return found;
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

/// Loads a tool URDF, through the view when one is attached.
static compages::world::Entity loadTool(compages::world::World& p_world,
                                        std::filesystem::path const& p_urdf,
                                        SceneView* p_view)
{
    if (p_view != nullptr)
    {
        return p_view->model(p_world, p_urdf);
    }
    auto loaded = compages::world::loadUrdf(p_world, p_urdf);
    if (!loaded)
    {
        throw std::runtime_error(loaded.error());
    }
    return loaded.value();
}

/// Parents the mount link of the tool on the flange link of the robot.
static void mountTool(compages::world::World& p_world,
                      compages::world::Entity p_robot,
                      ToolMount const& p_tool,
                      SceneView* p_view)
{
    compages::world::Entity const wrapper =
        loadTool(p_world, p_tool.urdf, p_view);
    compages::world::Entity const mount = findNamed(wrapper, p_tool.mount);
    compages::world::Entity const flange = findNamed(p_robot, p_tool.flange);
    if (!mount || !flange)
    {
        throw std::runtime_error("Tool '" + p_tool.urdf.string() +
                                 "' must hang from " + p_tool.flange +
                                 " by " + p_tool.mount);
    }
    if (!mount.setParent(flange))
    {
        throw std::runtime_error("Tool '" + p_tool.urdf.string() +
                                 "' could not be mounted on " +
                                 p_tool.flange);
    }
    // The loader's root only carries the Z-up to Y-up rotation. The mount
    // now hangs from the flange, which already lives in that frame.
    if (wrapper.id() != mount.id())
    {
        wrapper.destroy();
    }
}

/// Loads the URDF of the robot into the world, through the view when one is
/// attached.
static compages::world::Entity loadRobot(compages::world::World& p_world,
                                         std::filesystem::path const& p_urdf,
                                         SceneView* p_view)
{
    if (p_view != nullptr)
    {
        return p_view->robot(p_world, p_urdf);
    }
    auto loaded = compages::world::loadUrdf(p_world, p_urdf);
    if (!loaded)
    {
        throw std::runtime_error(loaded.error());
    }
    return loaded.value();
}

/// Appends to @p_joints the joint carried by @p_link, with the type and the
/// limits of its Compages joint component (read from the URDF).
static void addJoint(JointSet& p_joints,
                     std::string const& p_name,
                     compages::world::Entity p_link)
{
    if (auto const* joint = p_link.find<compages::world::RevoluteJoint>())
    {
        auto const& state = joint->state;
        bool const continuous = std::isinf(state.position.min.value()) &&
                                std::isinf(state.position.max.value());
        (void)p_joints.add(
            p_name,
            RevoluteLimits{ Radians(state.position.min.value()),
                            Radians(state.position.max.value()),
                            AngularVelocity(state.velocity.max.value()),
                            Torque(state.effort.max.value()) },
            continuous);
        return;
    }
    if (auto const* joint = p_link.find<compages::world::PrismaticJoint>())
    {
        auto const& state = joint->state;
        (void)p_joints.add(
            p_name,
            PrismaticLimits{ Length(state.position.min.value()),
                             Length(state.position.max.value()),
                             LinearVelocity(state.velocity.max.value()),
                             Force(state.effort.max.value()) });
        return;
    }
    throw std::invalid_argument("Link of joint '" + p_name +
                                "' has no revolute nor prismatic joint");
}

RobotRuntime::RobotRuntime(compages::world::World& p_world,
                           std::filesystem::path const& p_urdf,
                           SceneView* p_view,
                           ToolMount const& p_tool,
                           std::shared_ptr<ResourceManager> p_resources)
    : m_world(p_world),
      m_root(loadRobot(p_world, p_urdf, p_view)),
      m_world_from_urdf(m_root.rotation()),
      m_robot(p_urdf, p_tool, std::move(p_resources))
{
    UrdfRobot const urdf = readUrdf(p_urdf);
    Robot::Model model;
    model.name = urdf.name;
    model.urdf = p_urdf;
    model.tool_urdf = p_tool.urdf;
    m_robot.m_id = model.name;
    m_root.add<ecs::RobotTag>();
    m_root.set(ecs::RobotIdentity{ model.name, p_urdf });

    Kinematics const& kinematics = m_robot.m_kinematics;
    std::string tcp;
    std::string tool0;
    std::string end_effector;
    // Moving joints of a chain join this robot. A mounted tool therefore adds
    // its own axes (a finger, a spindle).
    auto adopt = [&](UrdfRobot const& p_chain, bool p_frames)
    {
        for (UrdfJoint const& joint : p_chain.joints)
        {
            if (!moving(joint.type))
            {
                if (!p_frames)
                {
                    continue;
                }
                if (joint.child == "tcp")
                {
                    tcp = joint.child;
                }
                else if (joint.child == "tool0")
                {
                    tool0 = joint.child;
                }
                else if (joint.child == "end_effector")
                {
                    end_effector = joint.child;
                }
                continue;
            }
            compages::world::Entity link = findNamed(m_root, joint.child);
            if (!link)
            {
                throw std::runtime_error("Compages has no link '" +
                                         joint.child + "' for joint '" +
                                         joint.name + "'");
            }
            addJoint(m_robot.m_joints, joint.name, link);
            m_links.push_back(link);
            model.joints.push_back(joint.name);
            model.q_indices.push_back(kinematics.qIndex(joint.name));
            model.v_indices.push_back(kinematics.vIndex(joint.name));
            if (p_frames)
            {
                model.tool = joint.child;
            }
        }
    };
    adopt(urdf, true);
    if (!tcp.empty())
    {
        model.tool = tcp;
    }
    else if (!tool0.empty())
    {
        model.tool = tool0;
    }
    else if (!end_effector.empty())
    {
        model.tool = end_effector;
    }
    if (hasTool(p_tool))
    {
        mountTool(p_world, m_root, p_tool, p_view);
        if (compages::world::Entity const tip = findNamed(m_root, "tcp"))
        {
            model.tool = tip.name();
        }
        adopt(readUrdf(p_tool.urdf), false);
    }
    m_robot.m_model = std::move(model);
    propagate();
}

RobotRuntime::~RobotRuntime() = default;

compages::world::Entity RobotRuntime::getLink(std::string_view p_name) const
{
    return findNamed(m_root, p_name);
}

Pose RobotRuntime::getTrueWorldPose(LinkRef const& p_link) const
{
    return m_robot.poseFrom(m_true_base, p_link);
}

Pose RobotRuntime::requireTrueWorldPose(std::string const& p_frame) const
{
    return p_frame.empty()
               ? m_true_base.pose
               : m_true_base.pose * m_robot.requireFramePose(p_frame);
}

void RobotRuntime::propagate()
{
    m_robot.updateKinematics();
    publishJoints();
    m_root.position(m_world_from_urdf * toFloat(m_true_base.pose.position))
        .rotation(m_world_from_urdf * toFloat(m_true_base.pose.rotation));
    m_root.set(m_true_base);
}

void RobotRuntime::publishJoints() const
{
    JointSet const& joints = m_robot.m_joints;
    for (JointId id = 0; id < joints.size(); ++id)
    {
        compages::world::Entity link = m_links[id];
        if (auto* revolute = link.find<compages::world::RevoluteJoint>())
        {
            revolute->state.position.value = Radians(joints.position(id));
            revolute->state.velocity.value =
                AngularVelocity(joints.velocity(id));
            revolute->state.effort.value = Torque(joints.effort(id));
        }
        else if (auto* prismatic = link.find<compages::world::PrismaticJoint>())
        {
            prismatic->state.position.value = Length(joints.position(id));
            prismatic->state.velocity.value =
                LinearVelocity(joints.velocity(id));
            prismatic->state.effort.value = Force(joints.effort(id));
        }
    }
}

void RobotRuntime::connect(std::unique_ptr<RobotBackend> p_backend)
{
    m_backend = std::move(p_backend);
    m_physics = dynamic_cast<PhysicsWorld*>(m_backend.get());
    if (m_backend)
    {
        m_backend->attach(*this);
        m_backend->reset(*this);
    }
}

void RobotRuntime::hold(JointPosture const& p_posture)
{
    JointSet& joints = m_robot.m_joints;
    for (auto const& [name, angle] : p_posture.angles)
    {
        JointId const id = joints.require(name);
        if (joints.type(id) == JointType::Prismatic)
        {
            throw std::invalid_argument("Joint '" + name +
                                        "' slides: give it a Length");
        }
        joints.home(id, angle.value());
        joints.place(id, angle.value());
    }
    for (auto const& [name, length] : p_posture.lengths)
    {
        JointId const id = joints.require(name);
        if (joints.type(id) != JointType::Prismatic)
        {
            throw std::invalid_argument("Joint '" + name +
                                        "' turns: give it Radians");
        }
        joints.home(id, length.value());
        joints.place(id, length.value());
    }
    m_robot.m_joints.hold();
    if (m_backend)
    {
        m_backend->reset(*this);
    }
    propagate();
    m_world.update();
}

void RobotRuntime::setStartPose(Pose const& p_pose)
{
    m_start = p_pose;
    m_robot.m_base = BaseState{ p_pose, {} };
    m_true_base = m_robot.m_base;
    if (m_backend)
    {
        m_backend->reset(*this);
    }
    propagate();
}

void RobotRuntime::reset()
{
    m_time = Seconds{};
    m_robot.m_base = BaseState{ m_start, {} };
    m_true_base = m_robot.m_base;
    for (JointId id = 0; id < m_robot.m_joints.size(); ++id)
    {
        m_robot.m_joints.place(id, m_robot.m_joints.home(id));
    }
    if (m_backend)
    {
        m_backend->reset(*this);
    }
    for (std::size_t i = 0; i < m_robot.m_sensors.size(); ++i)
    {
        m_robot.m_sensors[i].rewind();
    }
    propagate();
    m_world.update();
}

void RobotRuntime::step(Seconds p_dt)
{
    // A step moves the clock forward; zero would loop a run forever.
    if (!(p_dt.value() > 0.0) || !std::isfinite(p_dt.value()))
    {
        throw std::invalid_argument("RobotRuntime::step: dt must be positive "
                                    "and finite, not " +
                                    std::to_string(p_dt.value()));
    }
    prepare(p_dt);
    advance(p_dt);
    sense();
}

void RobotRuntime::prepare(Seconds /*p_dt*/)
{
    // A safe state is written whoever holds the actuator.
    WriterScope const safety(*m_robot.m_resources, SAFETY);
    for (std::size_t i = 0; i < m_robot.m_actuators.size(); ++i)
    {
        if (!m_robot.m_actuators.available(i))
        {
            m_robot.m_actuators[i].disable(m_robot);
        }
    }
}

void RobotRuntime::advance(Seconds p_dt)
{
    std::optional<Seconds> clock;
    if (m_backend)
    {
        m_backend->step(*this, p_dt);
        clock = m_backend->time();
    }
    checkHealth();
    // The backend gives the clock when it has one (simulated time, hardware
    // stamp); without one, the robot counts the steps.
    m_time = clock ? *clock : m_time + p_dt;
}

void RobotRuntime::checkHealth()
{
    m_health = m_backend ? m_backend->getHealth() : RobotBackend::Health{};
    ResourceManager& resources = *m_robot.m_resources;
    ActuatorSet const& actuators = m_robot.m_actuators;
    if (!m_health.ok && m_health_failed.empty())
    {
        // Lost: the actuators are failed resources, so the next prepare
        // puts them in their safe state and the skills see a fault.
        for (std::size_t i = 0; i < actuators.size(); ++i)
        {
            ResourceId const id = actuators.resource(i);
            if (resources.available(id))
            {
                resources.fail(id);
                m_health_failed.push_back(id);
            }
        }
    }
    else if (m_health.ok && !m_health_failed.empty())
    {
        // Back: only what the loss failed comes back (a fault of a scenario
        // on the same actuator stays).
        for (ResourceId const id : m_health_failed)
        {
            resources.restore(id);
        }
        m_health_failed.clear();
    }
}

void RobotRuntime::sense()
{
    propagate();
    m_world.update();

    for (std::size_t i = 0; i < m_robot.m_sensors.size(); ++i)
    {
        if (m_robot.m_sensors.available(i))
        {
            m_robot.m_sensors[i].update(*this, m_time);
        }
        else
        {
            m_robot.m_sensors[i].invalidate();
        }
    }
}

} // namespace robotik
