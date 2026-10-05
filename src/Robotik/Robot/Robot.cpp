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
    JointLimits limits;
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
        UrdfJoint joint;
        joint.name = node.attribute("name").as_string();
        joint.type = node.attribute("type").as_string();
        joint.child = node.child("child").attribute("link").as_string();
        if (pugi::xml_node limit = node.child("limit"))
        {
            joint.limits.lower = limit.attribute("lower").as_double(0.0);
            joint.limits.upper = limit.attribute("upper").as_double(0.0);
            joint.limits.velocity = limit.attribute("velocity").as_double(0.0);
            joint.limits.effort = limit.attribute("effort").as_double(0.0);
        }
        result.joints.push_back(std::move(joint));
    }
    return result;
}

std::optional<JointType> jointType(std::string_view p_type)
{
    if (p_type == "revolute")
    {
        return JointType::Revolute;
    }
    if (p_type == "continuous")
    {
        return JointType::Continuous;
    }
    if (p_type == "prismatic")
    {
        return JointType::Prismatic;
    }
    return std::nullopt;
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

} // namespace

Robot::Robot(compages::world::World& p_world,
             std::filesystem::path const& p_urdf,
             SceneView* p_view)
    : m_world(p_world),
      m_kinematics(std::make_unique<PinocchioBackend>(p_urdf)),
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

    UrdfRobot const urdf = readUrdf(p_urdf);
    m_name = urdf.name;
    m_root.add<ecs::RobotTag>();
    m_root.set(ecs::RobotIdentity{ m_name, p_urdf });

    bool named_tool = false;
    for (UrdfJoint const& joint : urdf.joints)
    {
        std::optional<JointType> const type = jointType(joint.type);
        if (!type)
        {
            if (joint.child == "tool0" || joint.child == "end_effector" ||
                joint.child == "tcp")
            {
                m_tool = joint.child;
                named_tool = true;
            }
            continue;
        }
        compages::world::Entity link = findNamed(m_root, joint.child);
        if (!link)
        {
            throw std::runtime_error("Compages has no link '" + joint.child +
                                     "' for joint '" + joint.name + "'");
        }
        m_joints.add(joint.name, *type, joint.limits, link);
        m_q_indices.push_back(m_kinematics->qIndex(joint.name));
        m_v_indices.push_back(m_kinematics->vIndex(joint.name));
        if (!named_tool)
        {
            m_tool = joint.child;
        }
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

    for (JointId id = 0; id < m_joints.size(); ++id)
    {
        compages::world::Entity link = m_joints.link(id);
        if (link.has<compages::world::PrismaticJoint>())
        {
            link.offset(units::length::meter_t(m_joints.position(id)));
        }
        else if (link.has<compages::world::RevoluteJoint>())
        {
            link.angle(units::angle::radian_t(m_joints.position(id)));
        }
    }
}

RobotSession::RobotSession(compages::world::World& p_world,
                           std::filesystem::path const& p_urdf,
                           SceneView* p_view)
    : Robot(p_world, p_urdf, p_view)
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

void RobotSession::reset()
{
    m_time = Seconds{};
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
