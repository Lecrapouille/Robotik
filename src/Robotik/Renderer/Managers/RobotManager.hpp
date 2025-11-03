/**
 * @file RobotManager.hpp
 * @brief OpenGL robot rendering class.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#pragma once

#include "Robotik/Core/Robot/Robot.hpp"
#include "Robotik/Core/Solvers/IKSolver.hpp"
#include "Robotik/Core/Solvers/Trajectory.hpp"

#include <Eigen/Dense>

#include <cstddef>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace robotik::renderer
{

// ****************************************************************************
//! \brief Robot manager class for handling multiple robot instances.
//!
//! This class manages basic Robot instances loaded from URDF files.
//! It provides methods to load, remove, and access robots by name.
//!
//! Key features:
//! - Load robots from URDF files
//! - Manage multiple robot instances
//! - Simple robot lifecycle management
//! - Error handling for robot operations
// ****************************************************************************
class RobotManager
{
public:

    // ------------------------------------------------------------------------
    //! \brief Destructor.
    // ------------------------------------------------------------------------
    ~RobotManager() = default;

    // ------------------------------------------------------------------------
    //! \brief Reset the memory occupied by the robot manager.
    // ------------------------------------------------------------------------
    void clear();

    // ------------------------------------------------------------------------
    //! \brief Load a robot from URDF file.
    //! \param p_urdf_path Path to the URDF file.
    //! \return Pointer to robot, nullptr if failed.
    // ------------------------------------------------------------------------
    Robot* loadRobot(const std::string& p_urdf_path);

    // ------------------------------------------------------------------------
    //! \brief Add an existing robot instance.
    //! \param p_robot_name Unique name for the robot.
    //! \param p_robot Robot instance to add.
    //! \return true if successful.
    // ------------------------------------------------------------------------
    bool addRobot(const std::string& p_robot_name,
                  std::unique_ptr<Robot> p_robot);

    // ------------------------------------------------------------------------
    //! \brief Remove a robot by name.
    //! \param p_robot_name Name of the robot to remove.
    //! \return true if successful.
    // ------------------------------------------------------------------------
    bool removeRobot(const std::string& p_robot_name);

    // ------------------------------------------------------------------------
    //! \brief Check if a robot is loaded.
    //! \param p_robot_name Name of the robot.
    //! \return true if loaded.
    // ------------------------------------------------------------------------
    bool hasRobot(const std::string& p_robot_name) const;

    // ------------------------------------------------------------------------
    //! \brief Get robot by name.
    //! \param p_robot_name Name of the robot.
    //! \return Pointer to robot, nullptr if not found.
    // ------------------------------------------------------------------------
    Robot* getRobot(const std::string& p_robot_name);

    // ------------------------------------------------------------------------
    //! \brief Get the current robot (first loaded robot).
    //! \return Pointer to current robot, nullptr if no robot loaded.
    // ------------------------------------------------------------------------
    Robot* currentRobot() const;

    // ------------------------------------------------------------------------
    //! \brief Set robot joint values.
    //! \param p_robot_name Name of the robot.
    //! \param p_joint_values Vector of joint values.
    //! \return true if successful.
    // ------------------------------------------------------------------------
    bool setRobotJointValues(const std::string& p_robot_name,
                             const std::vector<double>& p_joint_values);

    // ------------------------------------------------------------------------
    //! \brief Get robot joint values.
    //! \param p_robot_name Name of the robot.
    //! \return Vector of joint values, empty if not found.
    // ------------------------------------------------------------------------
    std::vector<double>
    getRobotJointValues(const std::string& p_robot_name) const;

    // ------------------------------------------------------------------------
    //! \brief Set robot pose (position and orientation).
    //! \param p_robot_name Name of the robot.
    //! \param p_transform 4x4 transformation matrix.
    //! \return true if successful.
    // ------------------------------------------------------------------------
    bool setRobotPose(const std::string& p_robot_name,
                      const Eigen::Matrix4f& p_transform);

    // ------------------------------------------------------------------------
    //! \brief Update robot transforms (call after joint changes).
    //! \param p_robot_name Name of the robot to update.
    //! \return true if successful.
    // ------------------------------------------------------------------------
    bool updateRobotTransforms(const std::string& p_robot_name);

    // ------------------------------------------------------------------------
    //! \brief Update all robot transforms.
    // ------------------------------------------------------------------------
    void updateAllRobotTransforms();

    // ------------------------------------------------------------------------
    //! \brief Get all robots.
    //! \return Map of robot names and robot pointers.
    // ------------------------------------------------------------------------
    std::unordered_map<std::string, std::unique_ptr<Robot>> const&
    robots() const
    {
        return m_robots;
    }

    std::unordered_map<std::string, std::unique_ptr<Robot>>& robots()
    {
        return m_robots;
    }

    // ------------------------------------------------------------------------
    //! \brief Get the number of loaded robots.
    //! \return Number of loaded robots.
    // ------------------------------------------------------------------------
    size_t getRobotCount() const
    {
        return m_robots.size();
    }

    // ------------------------------------------------------------------------
    //! \brief Get the last error message.
    //! \return Error message.
    // ------------------------------------------------------------------------
    const std::string& error() const
    {
        return m_error;
    }

private:

    //! \brief Managed robots.
    std::unordered_map<std::string, std::unique_ptr<Robot>> m_robots;
    //! \brief Current robot.
    Robot* m_current_robot = nullptr;
    //! \brief Last error message.
    std::string m_error;
};

} // namespace robotik::renderer
