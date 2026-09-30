// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "Robotik/Model/RobotLoader.hpp"

#include "Robotik/Backends/MujocoBackend.hpp"
#include "Robotik/Backends/PinocchioBackend.hpp"
#include "Robotik/ECS/ActuatorComponents.hpp"
#include "Robotik/ECS/BackendComponents.hpp"
#include "Robotik/ECS/JointComponents.hpp"
#include "Robotik/ECS/RobotComponents.hpp"
#include "Robotik/Systems/JointProjectionSystem.hpp"
#include "Robotik/Systems/PinocchioSyncSystem.hpp"

#include "Compages/Renderer/Assets/UrdfLoader.hpp"
#include "Compages/Renderer/Scene.hpp"
#include "Compages/World/Entity.hpp"

#include <pugixml.hpp>

#include <cctype>
#include <stdexcept>
#include <string>
#include <vector>

namespace robotik
{

namespace
{

struct UrdfJoint
{
    std::string name;
    std::string type;
    std::string child;
    double lower = 0.0;
    double upper = 0.0;
    double max_velocity = 0.0;
    double max_effort = 0.0;
    bool limited = false;
};

bool actuated(std::string const& p_type)
{
    return p_type == "revolute" || p_type == "continuous" ||
           p_type == "prismatic";
}

bool contains(std::string const& p_text, std::string const& p_needle)
{
    auto lower = [](std::string p_value)
    {
        for (char& character : p_value)
        {
            character = static_cast<char>(
                std::tolower(static_cast<unsigned char>(character)));
        }
        return p_value;
    };
    return lower(p_text).find(lower(p_needle)) != std::string::npos;
}

std::vector<UrdfJoint> readJoints(std::string const& p_filename)
{
    pugi::xml_document document;
    pugi::xml_parse_result const parsed =
        document.load_file(p_filename.c_str());
    if (!parsed)
    {
        throw std::runtime_error("Failed to parse URDF '" + p_filename +
                                 "': " + parsed.description());
    }

    std::vector<UrdfJoint> joints;
    for (pugi::xml_node node : document.child("robot").children("joint"))
    {
        UrdfJoint joint;
        joint.name = node.attribute("name").as_string();
        joint.type = node.attribute("type").as_string();
        joint.child = node.child("child").attribute("link").as_string();
        if (pugi::xml_node limit = node.child("limit"))
        {
            joint.limited = true;
            joint.lower = limit.attribute("lower").as_double(0.0);
            joint.upper = limit.attribute("upper").as_double(0.0);
            joint.max_velocity = limit.attribute("velocity").as_double(0.0);
            joint.max_effort = limit.attribute("effort").as_double(0.0);
        }
        joints.push_back(std::move(joint));
    }
    return joints;
}

compages::world::Entity findNamed(compages::world::Entity p_root,
                                  std::string const& p_name)
{
    if (!p_root)
    {
        return {};
    }
    if (p_root.name() == p_name)
    {
        return p_root;
    }
    compages::world::Entity found;
    p_root.children(
        [&](compages::world::Entity p_child)
        {
            if (!found)
            {
                found = findNamed(p_child, p_name);
            }
        });
    return found;
}

std::string robotName(std::string const& p_filename)
{
    pugi::xml_document document;
    if (!document.load_file(p_filename.c_str()))
    {
        return {};
    }
    return document.child("robot").attribute("name").as_string();
}

} // namespace

void RobotLoader::instantiate(compages::world::World& p_world,
                              compages::renderer::Scene* p_scene,
                              PinocchioBackend& p_pinocchio,
                              MujocoBackend* p_mujoco,
                              std::string const& p_filename)
{
    compages::Result<compages::world::Entity> loaded =
        (p_scene != nullptr)
            ? p_scene->load(p_filename)
            : compages::renderer::loadUrdf(p_world, p_filename);
    if (!loaded)
    {
        throw std::runtime_error(loaded.error());
    }

    compages::world::Entity root = loaded.value();
    root.add<ecs::RobotTag>();
    root.set(ecs::RobotIdentity{ robotName(p_filename), p_filename });

    std::string end_effector_link;
    for (UrdfJoint const& joint : readJoints(p_filename))
    {
        if (!actuated(joint.type))
        {
            if (joint.child == "tool0" || joint.child == "end_effector" ||
                joint.child == "tcp")
            {
                end_effector_link = joint.child;
            }
            continue;
        }

        compages::world::Entity link = findNamed(root, joint.child);
        if (!link)
        {
            throw std::runtime_error("Compages has no link '" + joint.child +
                                     "' for joint '" + joint.name + "'");
        }

        link.set(ecs::Link{ joint.child });
        link.set(ecs::Joint{ joint.name });
        link.set(ecs::JointState{});
        link.set(ecs::JointCommand{});
        link.set(ecs::HomePosition{ 0.0 });
        link.set(ecs::PositionController{});
        link.set(ecs::ActuatorCommand{});

        ecs::JointLimits limits;
        if (joint.limited)
        {
            limits.lower = joint.lower;
            limits.upper = joint.upper;
            limits.max_velocity = joint.max_velocity;
            limits.max_effort = joint.max_effort;
        }
        link.set(limits);

        ecs::PinocchioJointBinding pinocchio_binding;
        pinocchio_binding.q_index = p_pinocchio.qIndex(joint.name);
        pinocchio_binding.v_index = p_pinocchio.vIndex(joint.name);
        if (p_pinocchio.hasJoint(joint.name))
        {
            pinocchio_binding.joint_id = p_pinocchio.jointId(joint.name);
        }
        link.set(pinocchio_binding);

        ecs::MujocoJointBinding mujoco_binding;
        if (p_mujoco != nullptr)
        {
            int const joint_id = p_mujoco->jointId(joint.name);
            mujoco_binding.joint_id = joint_id;
            mujoco_binding.qpos_index = p_mujoco->qposIndex(joint_id);
            mujoco_binding.qvel_index = p_mujoco->qvelIndex(joint_id);
            mujoco_binding.dof_index = p_mujoco->dofIndex(joint_id);
            int const actuator = p_mujoco->actuatorId(joint.name);
            if (actuator >= 0)
            {
                link.set(ecs::MujocoActuatorBinding{ actuator });
            }
        }
        link.set(mujoco_binding);

        if (contains(joint.name, "gripper") || contains(joint.name, "finger") ||
            contains(joint.child, "gripper") || contains(joint.child, "finger"))
        {
            ecs::Gripper gripper;
            gripper.min_opening = limits.lower;
            gripper.max_opening = limits.upper;
            link.set(gripper);
        }

        end_effector_link = joint.child;
    }

    if (!end_effector_link.empty())
    {
        if (compages::world::Entity tool = findNamed(root, end_effector_link))
        {
            tool.set(ecs::EndEffector{ end_effector_link });
            if (p_pinocchio.hasFrame(end_effector_link))
            {
                tool.set(ecs::PinocchioFrameBinding{
                    p_pinocchio.frameId(end_effector_link) });
            }
        }
    }

    if (p_mujoco != nullptr)
    {
        p_world.each<ecs::JointState, ecs::MujocoJointBinding>(
            [&](compages::world::Entity,
                ecs::JointState& p_state,
                ecs::MujocoJointBinding& p_binding)
            {
                if (p_binding.qpos_index >= 0)
                {
                    p_state.position = p_mujoco->qpos(p_binding.qpos_index);
                }
                if (p_binding.qvel_index >= 0)
                {
                    p_state.velocity = p_mujoco->qvel(p_binding.qvel_index);
                }
            });
    }

    PinocchioSyncSystem{}.update(p_world, p_pinocchio);
    p_pinocchio.updateKinematics();
    JointProjectionSystem{}.update(p_world);
    p_world.update();
}

} // namespace robotik
