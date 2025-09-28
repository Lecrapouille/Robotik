/**
 * @file RobotManager.cpp
 * @brief OpenGL robot rendering class implementation.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#include "Viewer/RobotManager.hpp"
#include "Robotik/Parser.hpp"

namespace robotik::viewer
{

// ----------------------------------------------------------------------------
void RobotManager::reset()
{
    m_robots.clear();
    m_current_robot = nullptr;
    m_error.clear();
}

// ----------------------------------------------------------------------------
bool RobotManager::loadRobot(const std::string& p_urdf_path)
{
    robotik::URDFParser parser;
    auto robot = parser.load(p_urdf_path);

    if (!robot)
    {
        m_error = "Failed to load robot from URDF: " + parser.getError();
        return false;
    }

    // Store the robot name before moving the robot
    std::string robot_name = robot->name();
    return addRobot(robot_name, std::move(robot));
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

    ControlledRobot controlled_robot;
    controlled_robot.robot = std::move(p_robot);
    controlled_robot.is_visible = true;
    controlled_robot.scale = 1.0f;

    // Add the robot to the map, fails if the robot already exists.
    auto [it, success] =
        m_robots.try_emplace(p_robot_name, std::move(controlled_robot));

    // Set the current robot if the robot was added successfully.
    if (success)
    {
        m_current_robot = &it->second;
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
RobotManager::ControlledRobot*
RobotManager::getRobot(const std::string& p_robot_name)
{
    auto it = m_robots.find(p_robot_name);
    if (it == m_robots.end())
    {
        return nullptr;
    }
    return &it->second;
}

// ----------------------------------------------------------------------------
RobotManager::ControlledRobot* RobotManager::getCurrentRobot() const
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

    if (!it->second.robot->setJointValues(p_joint_values))
    {
        m_error = "Failed to set joint values for robot '" + p_robot_name + "'";
        return false;
    }

    // Update transforms after joint changes
    return updateRobotTransforms(p_robot_name);
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

    return it->second.robot->jointValues();
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
#endif

// ----------------------------------------------------------------------------
bool RobotManager::setRobotVisibility(const std::string& p_robot_name,
                                      bool p_visible)
{
    auto it = m_robots.find(p_robot_name);
    if (it == m_robots.end())
    {
        m_error = "Robot '" + p_robot_name + "' not found";
        return false;
    }

    it->second.is_visible = p_visible;
    return true;
}

// ----------------------------------------------------------------------------
bool RobotManager::setRobotScale(const std::string& p_robot_name, float p_scale)
{
    auto it = m_robots.find(p_robot_name);
    if (it == m_robots.end())
    {
        m_error = "Robot '" + p_robot_name + "' not found";
        return false;
    }

    it->second.scale = p_scale;
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
void RobotManager::clear()
{
    m_robots.clear();
    m_error.clear();
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
    // In a full implementation, we would traverse the robot's scene graph
    // and compute the transformation for each link based on joint values
    // if (!it->second.link_transforms.empty())
    //{
    // Keep the current transform for now
    // This would be replaced with actual forward kinematics
    //}

    return true;
}

} // namespace robotik::viewer
