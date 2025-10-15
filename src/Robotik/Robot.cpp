/**
 * @file Robot.hpp
 * @brief Robot class - Representation of a complete robotic system.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#include "Robotik/Core/Robot.hpp"
#include "Robotik/Core/Conversions.hpp"
#include "Robotik/Core/Exception.hpp"
#include "Robotik/Core/Joint.hpp"
#include "Robotik/Core/Link.hpp"

#include <algorithm>
#include <functional>

namespace robotik
{

// ----------------------------------------------------------------------------
Robot::Robot(std::string_view const& p_name, scene::Node::Ptr p_root)
    : m_name(p_name)
{
    root(std::move(p_root));
}

// ----------------------------------------------------------------------------
void Robot::root(scene::Node::Ptr p_root)
{
    m_root_node = std::move(p_root);
    cacheSceneGraph();
}

// ----------------------------------------------------------------------------
void Robot::cacheSceneGraph()
{
    // If the cache is not dirty, we can return early
    if (!m_scene_graph_cache_dirty)
    {
        return;
    }

    m_joints.clear();
    m_joints_map.clear();
    m_links_map.clear();
    m_sensors_map.clear();
    m_actuators_map.clear();

    if (m_root_node != nullptr)
    {
        m_root_node->traverse(
            [this](scene::Node& p_node, size_t /*p_depth*/)
            {
                // Only cache actuable joints
                if (auto joint = dynamic_cast<Joint*>(&p_node))
                {
                    if (joint->type() != Joint::Type::FIXED)
                    {
                        m_joints_map.try_emplace(joint->name(),
                                                 std::ref(*joint));
                        m_joints.emplace_back(std::ref(*joint));
                    }
                }
                // Cache links, note that their geometry and collision is given
                // in their API
                else if (auto link = dynamic_cast<Link*>(&p_node))
                {
                    m_links_map.try_emplace(link->name(), std::ref(*link));
                }
                // Cache sensors
                else if (auto sensor = dynamic_cast<Sensor*>(&p_node))
                {
                    m_sensors_map.try_emplace(sensor->name(),
                                              std::ref(*sensor));
                }
                // Cache actuators
                else if (auto actuator = dynamic_cast<Actuator*>(&p_node))
                {
                    m_actuators_map.try_emplace(actuator->name(),
                                                std::ref(*actuator));
                }
                else
                {
                    // Ignore other nodes
                }
            });
    }

    m_scene_graph_cache_dirty = false;
}

// ----------------------------------------------------------------------------
Pose Robot::calculatePoseError(Pose const& p_target_pose,
                               Pose const& p_current_pose) const
{
    Pose error;

    // Position error is straightforward
    error.head<3>() = p_target_pose.head<3>() - p_current_pose.head<3>();

    // Orientation error: convert to rotation matrices for proper subtraction
    Eigen::Vector3d target_euler = p_target_pose.tail<3>();
    Eigen::Vector3d current_euler = p_current_pose.tail<3>();

    Eigen::Matrix3d target_rot = utils::eulerToRotation(
        target_euler(0), target_euler(1), target_euler(2));
    Eigen::Matrix3d current_rot = utils::eulerToRotation(
        current_euler(0), current_euler(1), current_euler(2));

    // Compute rotation error: R_error = R_target * R_current^T
    Eigen::Matrix3d error_rot = target_rot * current_rot.transpose();

    // Convert back to Euler angles (axis-angle would be better but this is
    // simpler)
    Eigen::Vector3d error_euler = utils::rotationToEuler(error_rot);
    error.tail<3>() = error_euler;

    return error;
}

// ----------------------------------------------------------------------------
Jacobian Robot::jacobian(scene::Node const& p_end_effector) const
{
    const size_t num_joints = m_joints.size();
    Jacobian J(6, num_joints);

    // Position of the end effector
    Transform end_effector_transform = p_end_effector.worldTransform();
    Eigen::Vector3d end_pos = end_effector_transform.block<3, 1>(0, 3);

    for (size_t i = 0; i < num_joints; ++i)
    {
        Joint const& joint = m_joints[i].get();

        // Transformation of the joint in the global space
        Transform joint_transform = joint.worldTransform();

        // Position of the joint
        Eigen::Vector3d joint_pos = joint_transform.block<3, 1>(0, 3);

        // Orientation of the joint axis in the global space
        Eigen::Vector3d joint_axis =
            joint_transform.block<3, 3>(0, 0) * joint.axis();

        if ((joint.type() == Joint::Type::REVOLUTE) ||
            (joint.type() == Joint::Type::CONTINUOUS))
        {
            // Contribution to linear velocity: cross(axis, (end - joint))
            Eigen::Vector3d r = end_pos - joint_pos;
            Eigen::Vector3d v = joint_axis.cross(r);

            // Fill the Jacobian
            J.block<3, 1>(0, i) = v;
            J.block<3, 1>(3, i) = joint_axis;
        }
        else if (joint.type() == Joint::Type::PRISMATIC)
        {
            // Contribution to linear velocity: axis
            J.block<3, 1>(0, i) = joint_axis;
            J.block<3, 1>(3, i) = Eigen::Vector3d::Zero();
        }
    }

    return J;
}

// ----------------------------------------------------------------------------
Joint const& Robot::joint(std::string_view const& p_name) const
{
    auto const& it = m_joints_map.find(std::string(p_name));
    if (it == m_joints_map.end())
    {
        throw RobotikException("Joint not found: " + std::string(p_name));
    }
    return it->second.get();
}

// ----------------------------------------------------------------------------
Link const& Robot::link(std::string_view const& p_name) const
{
    auto const& it = m_links_map.find(std::string(p_name));
    if (it == m_links_map.end())
    {
        throw RobotikException("Link not found: " + std::string(p_name));
    }
    return it->second.get();
}

// ----------------------------------------------------------------------------
std::vector<double> Robot::jointPositions() const
{
    std::vector<double> values;
    values.reserve(m_joints.size());

    for (const auto& joint_ref : m_joints)
    {
        values.push_back(joint_ref.get().position());
    }

    return values;
}

// ----------------------------------------------------------------------------
std::vector<std::string> Robot::jointNames() const
{
    std::vector<std::string> names;
    names.reserve(m_joints.size());

    for (const auto& joint_ref : m_joints)
    {
        names.push_back(joint_ref.get().name());
    }

    return names;
}

// ----------------------------------------------------------------------------
std::vector<std::string> Robot::linkNames() const
{
    std::vector<std::string> names;
    names.reserve(m_links_map.size());
    for (const auto& [name, _] : m_links_map)
    {
        names.push_back(name);
    }
    return names;
}

// ----------------------------------------------------------------------------
std::vector<std::string> Robot::sensorNames() const
{
    std::vector<std::string> names;
    names.reserve(m_sensors_map.size());
    for (const auto& [name, _] : m_sensors_map)
    {
        names.push_back(name);
    }
    return names;
}

// ----------------------------------------------------------------------------
std::vector<std::string> Robot::actuatorNames() const
{
    std::vector<std::string> names;
    names.reserve(m_actuators_map.size());
    for (const auto& [name, _] : m_actuators_map)
    {
        names.push_back(name);
    }
    return names;
}

// ----------------------------------------------------------------------------
bool Robot::setJointValues(std::vector<double> const& p_values)
{
    if (p_values.size() != m_joints.size())
    {
        return false;
    }

    // Set all joint values first (avoid multiple transform updates)
    for (size_t i = 0; i < m_joints.size(); ++i)
    {
        m_joints[i].get().position(p_values[i]);
    }

    return true;
}

// ----------------------------------------------------------------------------
void Robot::setNeutralPosition()
{
    std::vector<double> neutral_joints(m_joints.size(), 0.0);
    setJointValues(neutral_joints);
}

// ----------------------------------------------------------------------------
size_t Robot::findEndEffectors(
    std::vector<std::reference_wrapper<Link const>>& p_end_effectors) const
{
    p_end_effectors.clear();
    p_end_effectors.reserve(m_links_map.size());

    for (const auto& [_, link_ref] : m_links_map)
    {
        const Link& link = link_ref.get();
        bool has_child_joints = false;
        for (const auto& child : link.children())
        {
            if (dynamic_cast<Joint const*>(child.get()))
            {
                has_child_joints = true;
                break;
            }
        }

        if (!has_child_joints)
        {
            p_end_effectors.emplace_back(std::ref(link));
        }
    }

    return p_end_effectors.size();
}

// ----------------------------------------------------------------------------
void Robot::setMeshPrefixPath(std::string const& p_prefix_path) const
{
    if (p_prefix_path.empty())
    {
        return;
    }

    for (auto const& [_, link] : m_links_map)
    {
        auto& geometry = link.get().geometry();

        if (geometry.type() == Geometry::Type::MESH)
        {
            geometry.meshPath(p_prefix_path + "/" + geometry.meshPath());
        }
    }
}

} // namespace robotik
