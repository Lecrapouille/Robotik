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
#include <filesystem>
#include <stdexcept>
#include <string>
#include <vector>

namespace robotik
{

//------------------------------------------------------------------------------
struct UrdfJoint
{
    std::string name;          //!< Joint name.
    std::string type;          //!< Joint type.
    std::string child;         //!< Child link name.
    double lower = 0.0;        //!< Lower limit.
    double upper = 0.0;        //!< Upper limit.
    double max_velocity = 0.0; //!< Maximum velocity.
    double max_effort = 0.0;   //!< Maximum effort.
    bool limited = false;      //!< True if the joint is limited.
};

//------------------------------------------------------------------------------
//! @brief Checks if the joint is actuated.
//! @param p_type Joint type.
//! @return True if the joint is actuated.
//------------------------------------------------------------------------------
static bool actuated(std::string_view const& p_type)
{
    return p_type == "revolute" || p_type == "continuous" ||
           p_type == "prismatic";
}

//------------------------------------------------------------------------------
//! @brief Checks if the text contains the needle.
//! @param p_text Text.
//! @param p_needle Needle.
//! @return True if the text contains the needle.
//------------------------------------------------------------------------------
static bool contains(std::string_view const& p_text,
                     std::string_view const& p_needle)
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
    return lower(std::string(p_text)).find(lower(std::string(p_needle))) !=
           std::string::npos;
}

//------------------------------------------------------------------------------
//! @brief Reads the joints from the URDF file.
//! @param p_urdf URDF file.
//! @return The joints.
//------------------------------------------------------------------------------
static std::vector<UrdfJoint> readJoints(std::filesystem::path const& p_urdf)
{
    pugi::xml_document document;
    pugi::xml_parse_result const parsed =
        document.load_file(p_urdf.string().c_str());
    if (!parsed)
    {
        throw std::runtime_error("Failed to parse URDF '" + p_urdf.string() +
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

//------------------------------------------------------------------------------
//! @brief Finds a named entity in the world.
//! @param p_root Root entity.
//! @param p_name Name.
//! @return The entity.
//------------------------------------------------------------------------------
static compages::world::Entity findNamed(compages::world::Entity p_root,
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
        [&found, &p_name](compages::world::Entity p_child)
        {
            if (!found)
            {
                found = findNamed(p_child, p_name);
            }
        });
    return found;
}

//------------------------------------------------------------------------------
//! @brief Reads the robot name from the URDF file.
//! @param p_urdf URDF file.
//! @return The robot name.
//------------------------------------------------------------------------------
static std::string robotName(std::filesystem::path const& p_urdf)
{
    pugi::xml_document document;
    if (!document.load_file(p_urdf.string().c_str()))
    {
        return {};
    }
    return document.child("robot").attribute("name").as_string();
}

//------------------------------------------------------------------------------
void RobotLoader::instantiate(compages::world::World& p_world,
                              compages::renderer::Scene* p_scene,
                              PinocchioBackend& p_pinocchio,
                              MujocoBackend const* p_mujoco,
                              std::filesystem::path const& p_urdf)
{
    // Load the URDF file
    std::string const urdf_path = p_urdf.string();
    compages::Result<compages::world::Entity> loaded =
        (p_scene != nullptr) ? p_scene->load(urdf_path)
                             : compages::renderer::loadUrdf(p_world, urdf_path);
    if (!loaded)
    {
        throw std::runtime_error(loaded.error());
    }

    // Create the robot entity
    compages::world::Entity root = loaded.value();
    root.add<ecs::RobotTag>();
    root.set(ecs::RobotIdentity{ robotName(p_urdf), p_urdf });

    // Find the end effector link
    std::string end_effector_link;
    for (UrdfJoint const& joint : readJoints(p_urdf))
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

        // Find the link
        compages::world::Entity link = findNamed(root, joint.child);
        if (!link)
        {
            throw std::runtime_error("Compages has no link '" + joint.child +
                                     "' for joint '" + joint.name + "'");
        }

        // Create the joint mechanism
        ecs::JointMechanism const mechanism =
            joint.type == "prismatic" ? ecs::JointMechanism::Prismatic
                                      : ecs::JointMechanism::Revolute;

        link.set(ecs::Link{ joint.child });
        link.set(ecs::Joint{ joint.name, mechanism });
        link.set(ecs::makeJointState(mechanism));
        link.set(ecs::makeJointCommand(mechanism));
        link.set(ecs::makeHomePosition(mechanism));
        link.set(ecs::PositionController{});
        link.set(ecs::ActuatorCommand{});

        // Create the joint limits
        if (joint.limited)
        {
            link.set(ecs::makeJointLimits(mechanism,
                                          joint.lower,
                                          joint.upper,
                                          joint.max_velocity,
                                          joint.max_effort));
        }
        else
        {
            link.set(ecs::makeJointLimits(mechanism, 0.0, 0.0, 0.0, 0.0));
        }

        // Create the Pinocchio joint binding
        ecs::PinocchioJointBinding pinocchio_binding;
        pinocchio_binding.q_index = p_pinocchio.qIndex(joint.name);
        pinocchio_binding.v_index = p_pinocchio.vIndex(joint.name);
        if (p_pinocchio.hasJoint(joint.name))
        {
            pinocchio_binding.joint_id = p_pinocchio.jointId(joint.name);
        }
        link.set(pinocchio_binding);

        // Create the MuJoCo joint binding
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

        // Set the gripper
        if (contains(joint.name, "gripper") || contains(joint.name, "finger") ||
            contains(joint.child, "gripper") || contains(joint.child, "finger"))
        {
            ecs::Gripper gripper;
            ecs::JointLimits const& limits = link.get<ecs::JointLimits>();
            gripper.min_opening = Length(ecs::limitLowerSi(limits));
            gripper.max_opening = Length(ecs::limitUpperSi(limits));
            link.set(gripper);
        }

        end_effector_link = joint.child;
    }

    // Set the end effector
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

    // Set the MuJoCo joint state
    if (p_mujoco != nullptr)
    {
        // Read the state from MuJoCo
        p_world.each<ecs::JointState, ecs::MujocoJointBinding>(
            [&p_mujoco](compages::world::Entity,
                        ecs::JointState& p_state,
                        ecs::MujocoJointBinding const& p_binding)
            {
                // Write the position to the state
                if (p_binding.qpos_index >= 0)
                {
                    ecs::setPosition(p_state,
                                     p_mujoco->qpos(p_binding.qpos_index));
                }

                // Write the velocity to the state
                if (p_binding.qvel_index >= 0)
                {
                    ecs::setVelocity(p_state,
                                     p_mujoco->qvel(p_binding.qvel_index));
                }
            });
    }

    // Update the kinematics and projection
    PinocchioSyncSystem{}.update(p_world, p_pinocchio);
    p_pinocchio.updateKinematics();
    JointProjectionSystem{}.update(p_world);
    p_world.update();
}

} // namespace robotik
