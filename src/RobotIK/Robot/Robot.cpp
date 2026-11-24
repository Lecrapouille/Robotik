// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "RobotIK/Robot/RobotSession.hpp"

#include "RobotIK/Robot/Backends/PinocchioBackend.hpp"
#include "RobotIK/Robot/Components/Robots.hpp"

#include "Compages/World/Import/UrdfLoader.hpp"
#include "Compages/World/World.hpp"

#include <pugixml.hpp>

#include <atomic>
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

/// Reads the robot name and its moving joints from a URDF.
/// A serial number never given before in this process (0 is "no robot").
static std::uint64_t nextSerial()
{
    static std::atomic<std::uint64_t> s_last{ 0 };
    return ++s_last;
}

static UrdfRobot readUrdf(std::filesystem::path const& p_urdf)
{
    pugi::xml_document document;
    pugi::xml_parse_result const parsed =
        document.load_file(p_urdf.string().c_str());
    if (!parsed)
    {
        throw std::runtime_error("Failed to parse URDF '" + p_urdf.string() +
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

/// True for a revolute, continuous or prismatic joint.
static bool moving(std::string_view p_type)
{
    return p_type == "revolute" || p_type == "continuous" ||
           p_type == "prismatic";
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

/// Parents the tool mount frame on the robot flange.
static void mountTool(compages::world::World& p_world,
                      compages::world::Entity p_robot,
                      std::filesystem::path const& p_tool,
                      SceneView* p_view)
{
    compages::world::Entity const wrapper = loadTool(p_world, p_tool, p_view);
    compages::world::Entity const mount = findNamed(wrapper, "tool_mount");
    compages::world::Entity const flange = findNamed(p_robot, "flange");
    if (!mount || !flange)
    {
        throw std::runtime_error("Tool '" + p_tool.string() +
                                 "' must hang from flange by tool_mount");
    }
    if (!mount.setParent(flange))
    {
        throw std::runtime_error("Tool '" + p_tool.string() +
                                 "' could not be mounted on flange");
    }
    // The loader's root only carries the Z-up to Y-up rotation. tool_mount
    // now hangs from the flange, which already lives in that frame.
    if (wrapper.id() != mount.id())
    {
        wrapper.destroy();
    }
}

Robot::Robot(compages::world::World& p_world,
             std::filesystem::path const& p_urdf,
             SceneView* p_view,
             std::filesystem::path const& p_tool,
             std::shared_ptr<ResourceManager> p_resources)
    : m_world(p_world),
      m_kinematics(std::make_unique<PinocchioBackend>(p_urdf, p_tool)),
      m_resources(p_resources ? std::move(p_resources)
                              : std::make_shared<ResourceManager>()),
      m_sensors(*this, *m_resources),
      m_actuators(*this, *m_resources),
      m_serial(nextSerial())
{
    if (p_view != nullptr)
    {
        m_root = p_view->robot(p_world, p_urdf);
    }
    else
    {
        auto loaded = compages::world::loadUrdf(p_world, p_urdf);
        if (!loaded)
        {
            throw std::runtime_error(loaded.error());
        }
        m_root = loaded.value();
    }
    m_world_from_urdf = m_root.rotation();

    UrdfRobot const urdf = readUrdf(p_urdf);
    m_name = urdf.name;
    m_id = m_name;
    m_root.add<ecs::RobotTag>();
    m_root.set(ecs::RobotIdentity{ m_name, p_urdf });

    std::string tcp;
    std::string tool0;
    std::string end_effector;
    // Moving joints of a chain join this robot. A mounted tool therefore adds
    // its own axes (a finger, a spindle). Detach drops the chain, and with it
    // those joints; attaching it again brings the same count back.
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
            m_joints.add(joint.name, link);
            m_q_indices.push_back(m_kinematics->qIndex(joint.name));
            m_v_indices.push_back(m_kinematics->vIndex(joint.name));
            if (p_frames)
            {
                m_tool = joint.child;
            }
        }
    };
    adopt(urdf, true);
    if (!tcp.empty())
    {
        m_tool = tcp;
    }
    else if (!tool0.empty())
    {
        m_tool = tool0;
    }
    else if (!end_effector.empty())
    {
        m_tool = end_effector;
    }
    if (!p_tool.empty())
    {
        mountTool(p_world, m_root, p_tool, p_view);
        if (compages::world::Entity const tip = findNamed(m_root, "tcp"))
        {
            m_tool = tip.name();
        }
        adopt(readUrdf(p_tool), false);
    }
    propagate();
}

Robot::~Robot() = default;

compages::world::Entity RobotSession::link(std::string_view p_name) const
{
    return findNamed(m_root, p_name);
}

Pose Robot::requireFramePose(std::string const& p_frame) const
{
    return m_kinematics->requireFramePose(p_frame);
}

Pose Robot::requireWorldPose(std::string const& p_frame) const
{
    return p_frame.empty() ? m_base.pose
                           : m_base.pose * requireFramePose(p_frame);
}

LinkRef Robot::requireLink(std::string const& p_name) const
{
    if (p_name.empty())
    {
        return LinkRef{ m_serial, LinkRef::BASE };
    }
    if (!m_kinematics->hasFrame(p_name))
    {
        throw std::invalid_argument("Unknown Pinocchio frame: " + p_name);
    }
    return LinkRef{ m_serial, m_kinematics->frameId(p_name) };
}

Pose Robot::worldPose(LinkRef const& p_link) const
{
    return poseFrom(m_base, p_link);
}

Pose Robot::poseFrom(BaseState const& p_base, LinkRef const& p_link) const
{
    if (p_link.m_robot != m_serial)
    {
        throw std::logic_error(
            "Robot '" + m_id +
            "': this LinkRef was not found on this robot (another robot, or "
            "never found): call requireLink on this robot");
    }
    return p_link.m_frame == LinkRef::BASE
               ? p_base.pose
               : p_base.pose * m_kinematics->framePose(p_link.m_frame);
}

Pose RobotSession::trueWorldPose(LinkRef const& p_link) const
{
    return poseFrom(m_true_base, p_link);
}

Pose RobotSession::requireTrueWorldPose(std::string const& p_frame) const
{
    return p_frame.empty() ? m_true_base.pose
                           : m_true_base.pose * requireFramePose(p_frame);
}

std::optional<std::vector<double>>
Robot::inverseKinematics(std::string const& p_frame,
                         Pose const& p_target,
                         IkGoal p_goal) const
{
    auto solution = m_kinematics->solveIK(
        p_frame, p_target, m_kinematics->configuration(), p_goal);
    if (!solution)
    {
        return std::nullopt;
    }
    std::vector<double> targets(m_joints.size());
    for (JointId id = 0; id < m_joints.size(); ++id)
    {
        int const index = m_q_indices[id];
        targets[id] = index >= 0 ? (*solution)[static_cast<std::size_t>(index)]
                                 : m_joints.position(id);
    }
    return targets;
}

void Robot::propagate()
{
    std::span<double> q = m_kinematics->configuration();
    std::span<double> v = m_kinematics->velocity();
    for (JointId id = 0; id < m_joints.size(); ++id)
    {
        if (int const index = m_q_indices[id]; index >= 0)
        {
            q[static_cast<std::size_t>(index)] = m_joints.position(id);
        }
        if (int const index = m_v_indices[id]; index >= 0)
        {
            v[static_cast<std::size_t>(index)] = m_joints.velocity(id);
        }
    }
    m_kinematics->updateKinematics();

    m_joints.publish();
    m_root.position(m_world_from_urdf * toFloat(m_true_base.pose.position))
        .rotation(m_world_from_urdf * toFloat(m_true_base.pose.rotation));
    m_root.set(m_true_base);
}

RobotSession::RobotSession(compages::world::World& p_world,
                           std::filesystem::path const& p_urdf,
                           SceneView* p_view,
                           std::filesystem::path const& p_tool,
                           std::shared_ptr<ResourceManager> p_resources)
    : Robot(p_world, p_urdf, p_view, p_tool, std::move(p_resources))
{
}

RobotSession::~RobotSession() = default;

void RobotSession::connect(std::unique_ptr<RobotBackend> p_backend)
{
    m_backend = std::move(p_backend);
    if (m_backend)
    {
        m_backend->attach(*this);
        m_backend->reset(*this);
    }
}

void RobotSession::hold(JointPosture const& p_posture)
{
    for (auto const& [name, position] : p_posture)
    {
        JointId const id = m_joints.require(name);
        m_joints.home(id, position);
        m_joints.place(id, position);
    }
    m_joints.hold();
    if (m_backend)
    {
        m_backend->reset(*this);
    }
    propagate();
    m_world.update();
}

void RobotSession::startPose(Pose const& p_pose)
{
    m_start = p_pose;
    m_base = BaseState{ p_pose, {} };
    m_true_base = m_base;
    if (m_backend)
    {
        m_backend->reset(*this);
    }
    propagate();
}

void RobotSession::reset()
{
    m_time = Seconds{};
    m_base = BaseState{ m_start, {} };
    m_true_base = m_base;
    for (JointId id = 0; id < m_joints.size(); ++id)
    {
        m_joints.place(id, m_joints.home(id));
    }
    if (m_backend)
    {
        m_backend->reset(*this);
    }
    for (std::size_t i = 0; i < m_sensors.size(); ++i)
    {
        m_sensors[i].rewind();
    }
    propagate();
    m_world.update();
}

void RobotSession::step(Seconds p_dt)
{
    // A step moves the clock forward; zero would loop a run forever.
    if (!(p_dt.value() > 0.0) || !std::isfinite(p_dt.value()))
    {
        throw std::invalid_argument("RobotSession::step: dt must be positive "
                                    "and finite, not " +
                                    std::to_string(p_dt.value()));
    }
    prepare(p_dt);
    advance(p_dt);
    sense();
}

void RobotSession::prepare(Seconds /*p_dt*/)
{
    for (std::size_t i = 0; i < m_actuators.size(); ++i)
    {
        if (!m_actuators.available(i))
        {
            m_actuators[i].disable(*this);
        }
    }
}

void RobotSession::advance(Seconds p_dt)
{
    std::optional<Seconds> clock;
    if (m_backend)
    {
        m_backend->step(*this, p_dt);
        clock = m_backend->time();
    }
    // The backend gives the clock when it has one (simulated time, hardware
    // stamp); without one, the robot counts the steps.
    m_time = clock ? *clock : m_time + p_dt;
}

void RobotSession::sense()
{
    propagate();
    m_world.update();

    for (std::size_t i = 0; i < m_sensors.size(); ++i)
    {
        if (m_sensors.available(i))
        {
            m_sensors[i].update(*this, m_time);
        }
        else
        {
            m_sensors[i].invalidate();
        }
    }
}

} // namespace robotik
