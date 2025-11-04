/**
 * @file RobotManager.cpp
 * @brief OpenGL robot rendering class implementation.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#include "Robotik/Renderer/Managers/RobotManager.hpp"
#include "Robotik/Core/Loaders/UrdfLoader.hpp"

namespace robotik::renderer
{

// ----------------------------------------------------------------------------
void RobotManager::clear()
{
    m_robots.clear();
    m_current_robot = nullptr;
    m_error.clear();
}

// ----------------------------------------------------------------------------
bool RobotManager::addRobot(const std::string& p_robot_name,
                            std::unique_ptr<Robot> p_robot)
{
    if (!p_robot)
    {
        m_error = "Robot instance is null";
        return false;
    }

    if (hasRobot(p_robot_name))
    {
        m_error = "Robot '" + p_robot_name + "' already exists";
        return false;
    }

    // Add the robot to the map
    auto [it, success] = m_robots.try_emplace(p_robot_name, std::move(p_robot));

    // Set the current robot if the robot was added successfully.
    if (success)
    {
        m_current_robot = it->second.get();
    }
    return success;
}

// ----------------------------------------------------------------------------
bool RobotManager::removeRobot(const std::string& p_robot_name)
{
    auto it = m_robots.find(p_robot_name);
    if (it == m_robots.end())
    {
        m_error = "Robot '" + p_robot_name + "' not found";
        return false;
    }

    m_robots.erase(it);
    return true;
}

// ----------------------------------------------------------------------------
bool RobotManager::hasRobot(const std::string& p_robot_name) const
{
    return m_robots.find(p_robot_name) != m_robots.end();
}

// ----------------------------------------------------------------------------
Robot* RobotManager::getRobot(const std::string& p_robot_name)
{
    auto it = m_robots.find(p_robot_name);
    if (it == m_robots.end())
    {
        return nullptr;
    }
    return it->second.get();
}

// ----------------------------------------------------------------------------
Robot* RobotManager::currentRobot() const
{
    return m_current_robot;
}

// ----------------------------------------------------------------------------
bool RobotManager::setRobotJointValues(
    const std::string& p_robot_name,
    const std::vector<double>& p_joint_values)
{
    auto it = m_robots.find(p_robot_name);
    if (it == m_robots.end())
    {
        m_error = "Robot '" + p_robot_name + "' not found";
        return false;
    }

    if (it->second->states().joint_positions.size() != p_joint_values.size())
    {
        m_error = "Failed to set joint values for robot '" + p_robot_name + "'";
        return false;
    }

    it->second->states().joint_positions = p_joint_values;
    return true;
}

// ----------------------------------------------------------------------------
std::vector<double>
RobotManager::getRobotJointValues(const std::string& p_robot_name) const
{
    auto it = m_robots.find(p_robot_name);
    if (it == m_robots.end())
    {
        return {};
    }

    return it->second->states().joint_positions;
}

#if 0
// ----------------------------------------------------------------------------
bool RobotManager::setRobotPose(const std::string& p_robot_name,
                                const Eigen::Matrix4f& p_transform)
{
    auto it = m_robots.find(p_robot_name);
    if (it == m_robots.end())
    {
        m_error = "Robot '" + p_robot_name + "' not found";
        return false;
    }

    // For now, we'll just store the transform
    // In a full implementation, we would apply this to the root of the robot
    if (!it->second.link_transforms.empty())
    {
        it->second.link_transforms[0] = p_transform;
    }

    return true;
}

// ----------------------------------------------------------------------------
bool RobotManager::updateRobotTransforms(const std::string& p_robot_name)
{
    return updateRobotLinkTransforms(p_robot_name);
}

// ----------------------------------------------------------------------------
void RobotManager::updateAllRobotTransforms()
{
    for (const auto& [name, visual] : m_robots)
    {
        updateRobotTransforms(name);
    }
}

// ----------------------------------------------------------------------------
bool RobotManager::updateRobotLinkTransforms(const std::string& p_robot_name)
{
    auto it = m_robots.find(p_robot_name);
    if (it == m_robots.end())
    {
        return false;
    }

    // For now, we'll just update the first link transform
    // In a full implementation, we would traverse the robot's kinematic tree
    // and compute the transformation for each link based on joint values
    // if (!it->second.link_transforms.empty())
    //{
    // Keep the current transform for now
    // This would be replaced with actual forward kinematics
    //}

    return true;
}
#endif

} // namespace robotik::renderer
