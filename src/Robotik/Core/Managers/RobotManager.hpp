/**
 * @file RobotManager.hpp
 * @brief OpenGL robot rendering class.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#pragma once

#include "Robotik/Core/Common/Signal.hpp"
#include "Robotik/Core/Loaders/RobotLoaderFactory.hpp"
#include "Robotik/Core/Robot/Robot.hpp"
#include "Robotik/Core/Solvers/IKSolver.hpp"

#include <Eigen/Dense>

#include <cstddef>
#include <memory>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <vector>

namespace robotik
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
    //! \brief Signal emitted when a robot is added.
    //! \param p_robot Pointer to the added robot.
    // ------------------------------------------------------------------------
    Signal<Robot*> onRobotAdded;

    // ------------------------------------------------------------------------
    //! \brief Signal emitted when a robot is removed.
    //! \param p_robot_name Name of the removed robot.
    // ------------------------------------------------------------------------
    Signal<std::string const&> onRobotRemoved;

    // ------------------------------------------------------------------------
    //! \brief Reset the memory occupied by the robot manager.
    // ------------------------------------------------------------------------
    void clear();

    // ------------------------------------------------------------------------
    //! \brief Load a robot from URDF file.
    //! \param p_urdf_path Path to the URDF file.
    //! \return Pointer to robot, nullptr if failed.
    // ------------------------------------------------------------------------
    template <typename RobotType = Robot>
    RobotType* loadRobot(const std::string& p_urdf_path)
    {
        static_assert(std::is_base_of_v<Robot, RobotType>,
                      "RobotType must inherit from Robot");
        auto loader = RobotLoaderFactory::create(p_urdf_path);
        if (!loader)
        {
            m_error = "Failed to create loader for: " + p_urdf_path;
            return nullptr;
        }

        auto robot = loader->loadAs<RobotType>(p_urdf_path);
        if (!robot)
        {
            m_error = "Failed to load robot from: " + p_urdf_path + " - " +
                      loader->error();
            return nullptr;
        }

        // Get robot name before moving
        std::string robot_name = robot->name();

        // Store the new robot
        if (!addRobot(robot_name, std::move(robot)))
        {
            m_error = "Failed to add robot: " + m_error;
            return nullptr;
        }

        // Return the new robot
        return static_cast<RobotType*>(m_current_robot);
    }

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
    //! \brief Get robot by name (template version for derived types).
    //! \tparam RobotType Type of robot (default: Robot).
    //! \param p_robot_name Name of the robot.
    //! \return Pointer to robot of specified type, nullptr if not found.
    // ------------------------------------------------------------------------
    template <typename RobotType = Robot>
    RobotType* getRobot(const std::string& p_robot_name)
    {
        static_assert(std::is_base_of_v<Robot, RobotType>,
                      "RobotType must inherit from Robot");
        auto it = m_robots.find(p_robot_name);
        if (it == m_robots.end())
        {
            return nullptr;
        }
        return static_cast<RobotType*>(it->second.get());
    }

    // ------------------------------------------------------------------------
    //! \brief Get the current robot (first loaded robot).
    //! \return Pointer to current robot, nullptr if no robot loaded.
    // ------------------------------------------------------------------------
    Robot* currentRobot() const;

    // ------------------------------------------------------------------------
    //! \brief Get the current robot (template version for derived types).
    //! \tparam RobotType Type of robot (default: Robot).
    //! \return Pointer to current robot of specified type, nullptr if no robot
    //! loaded.
    // ------------------------------------------------------------------------
    template <typename RobotType = Robot>
    RobotType* currentRobot() const
    {
        static_assert(std::is_base_of_v<Robot, RobotType>,
                      "RobotType must inherit from Robot");
        return static_cast<RobotType*>(m_current_robot);
    }

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

    //! \brief Robot loader factory.
    RobotLoaderFactory m_loader_factory;
    //! \brief Managed robots.
    std::unordered_map<std::string, std::unique_ptr<Robot>> m_robots;
    //! \brief Current robot.
    Robot* m_current_robot = nullptr;
    //! \brief Last error message.
    std::string m_error;
};

} // namespace robotik
