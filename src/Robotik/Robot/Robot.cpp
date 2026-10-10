// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "Robotik/Robot/Robot.hpp"

#include "Robotik/Backends/PinocchioBackend.hpp"
#include "Robotik/ECS/RobotComponents.hpp"

#include "Compages/Renderer/Assets/UrdfLoader.hpp"
#include "Compages/World/World.hpp"

#include <pugixml.hpp>

#include <stdexcept>

namespace robotik
{

namespace
{

struct UrdfJoint
{
    std::string name;
    std::string type;
    std::string child;
};

struct UrdfRobot
{
    std::string name;
    std::vector<UrdfJoint> joints;
};

UrdfRobot readUrdf(std::filesystem::path const& p_urdf)
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
        result.joints.push_back({ node.attribute("name").as_string(),
                                  node.attribute("type").as_string(),
                                  node.child("child").attribute("link").as_string() });
    }
    return result;
}

bool moving(std::string_view p_type)
{
    return p_type == "revolute" || p_type == "continuous" ||
           p_type == "prismatic";
}

compages::world::Entity findNamed(compages::world::Entity p_root,
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

compages::core::Vector3f toFloat(Vector3 const& p_v)
{
    return { static_cast<float>(p_v.x),
             static_cast<float>(p_v.y),
             static_cast<float>(p_v.z) };
}

compages::core::Quatf toFloat(Quaternion const& p_q)
{
    return { static_cast<float>(p_q.w),
             static_cast<float>(p_q.x),
             static_cast<float>(p_q.y),
             static_cast<float>(p_q.z) };
}

compages::world::Entity loadTool(compages::world::World& p_world,
                                 std::filesystem::path const& p_urdf,
                                 SceneView* p_view)
{
    if (p_view != nullptr)
    {
        return p_view->model(p_world, p_urdf);
    }
    auto loaded = compages::renderer::loadUrdf(p_world, p_urdf.string());
    if (!loaded)
    {
        throw std::runtime_error(loaded.error());
    }
    return loaded.value();
}

void mountTool(compages::world::World& p_world,
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

} // namespace

Robot::Robot(compages::world::World& p_world,
             std::filesystem::path const& p_urdf,
             SceneView* p_view,
             std::filesystem::path const& p_tool)
    : m_world(p_world),
      m_kinematics(std::make_unique<PinocchioBackend>(p_urdf, p_tool)),
      m_sensors(*this, m_resources),
      m_actuators(*this, m_resources)
{
    if (p_view != nullptr)
    {
        m_root = p_view->robot(p_world, p_urdf);
    }
    else
    {
        auto loaded = compages::renderer::loadUrdf(p_world, p_urdf.string());
        if (!loaded)
        {
            throw std::runtime_error(loaded.error());
        }
        m_root = loaded.value();
    }
    m_world_from_urdf = m_root.rotation();

    UrdfRobot const urdf = readUrdf(p_urdf);
    m_name = urdf.name;
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
                throw std::runtime_error("Compages has no link '" + joint.child +
                                         "' for joint '" + joint.name + "'");
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
        if (compages::world::Entity const tip = link("tcp"))
        {
            m_tool = tip.name();
        }
        adopt(readUrdf(p_tool), false);
    }
    propagate();
}

Robot::~Robot() = default;

compages::world::Entity Robot::link(std::string_view p_name) const
{
    return findNamed(m_root, p_name);
}

Pose Robot::framePose(std::string const& p_frame) const
{
    return m_kinematics->framePose(p_frame);
}

Pose Robot::worldPose(std::string const& p_frame) const
{
    return p_frame.empty() ? m_base.pose : m_base.pose * framePose(p_frame);
}

std::optional<std::vector<double>>
Robot::inverseKinematics(std::string const& p_frame, Pose const& p_target) const
{
    auto solution =
        m_kinematics->solveIK(p_frame, p_target, m_kinematics->configuration());
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
    m_root.position(m_world_from_urdf * toFloat(m_base.pose.position))
        .rotation(m_world_from_urdf * toFloat(m_base.pose.rotation));
    m_root.set(m_base);
}

RobotSession::RobotSession(compages::world::World& p_world,
                           std::filesystem::path const& p_urdf,
                           SceneView* p_view,
                           std::filesystem::path const& p_tool)
    : Robot(p_world, p_urdf, p_view, p_tool)
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
    for (std::size_t i = 0; i < m_actuators.size(); ++i)
    {
        if (!m_actuators.available(i))
        {
            m_actuators[i].disable(*this);
        }
    }

    m_time = m_time + p_dt;
    if (m_backend)
    {
        m_backend->step(*this, p_dt);
    }
    propagate();
    m_world.update();

    for (std::size_t i = 0; i < m_sensors.size(); ++i)
    {
        if (m_sensors.available(i))
        {
            m_sensors[i].update(*this, m_time);
        }
    }
}

} // namespace robotik
