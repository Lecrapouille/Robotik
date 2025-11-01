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
//! \brief OpenGL robot rendering class.
//!
//! This class handles the rendering of robot models loaded from URDF files.
//! It manages the visual representation of robot links, joints, and provides
//! methods to update the robot's pose and render it in 3D space.
//!
//! Key features:
//! - Renders robot links as 3D geometries (boxes, cylinders, spheres, meshes)
//! - Supports joint transformations and animations
//! - Manages robot materials and colors
//! - Provides coordinate frame visualization
//! - Handles multiple robot instances
// ****************************************************************************
class RobotManager
{
public:

    // ------------------------------------------------------------------------
    //! \brief Control modes for the robot.
    // ------------------------------------------------------------------------
    enum class ControlMode
    {
        //! No control
        NO_CONTROL,
        //! Direct kinematics - manual joint control
        DIRECT_KINEMATICS,
        //! Automatic sinusoidal animation
        ANIMATION,
        //! Interactive inverse kinematics control
        INVERSE_KINEMATICS,
        //! Trajectory following with velocity limits
        TRAJECTORY
    };

    // ------------------------------------------------------------------------
    //! \brief Controlled robot class that extends Robot with rendering and
    //! control capabilities.
    // ------------------------------------------------------------------------
    class ControlledRobot: public Robot
    {
    public:

        //! \brief Constructor forwarding to Robot constructor
        ControlledRobot(std::string const& p_name, Blueprint&& p_blueprint)
            : Robot(p_name, std::move(p_blueprint))
        {
        }

        //! \brief Control mode (NO_CONTROL, ANIMATION, INVERSE_KINEMATICS,
        //! TRAJECTORY)
        ControlMode control_mode = ControlMode::NO_CONTROL;
        //! \brief Controlled link index for Inverse Kinematics (flat array
        //! architecture)
        size_t control_link_index = 0;
        //! \brief Computed target poses for IK (3 poses)
        std::vector<robotik::Pose> ik_target_poses;
        //! \brief IK solver instance
        std::unique_ptr<robotik::IKSolver> ik_solver;
        //! \brief Tracked link index for camera (flat array architecture)
        size_t camera_target_link_index = 0;
        //! \brief Camera tracking enabled
        bool camera_tracking_enabled = true;
        //! \brief Visibility flag
        bool is_visible = true;
        //! \brief Scale factor
        float scale = 1.0f;
        //! \brief Current trajectory (for TRAJECTORY mode)
        std::unique_ptr<robotik::Trajectory> trajectory;
        //! \brief Trajectory start time
        double trajectory_start_time = 0.0;
        //! \brief Trajectory configurations (start and goal)
        std::vector<std::vector<double>> trajectory_configs;
        //! \brief Current trajectory segment index
        size_t trajectory_segment = 0;
    };

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
    //! \return Pointer to controlled robot, nullptr if failed.
    // ------------------------------------------------------------------------
    ControlledRobot* loadRobot(const std::string& p_urdf_path);

    // ------------------------------------------------------------------------
    //! \brief Add an existing controlled robot instance.
    //! \param p_robot_name Unique name for the robot.
    //! \param p_robot Controlled robot instance to add.
    //! \return true if successful.
    // ------------------------------------------------------------------------
    bool addRobot(const std::string& p_robot_name,
                  std::unique_ptr<ControlledRobot> p_robot);

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
    RobotManager::ControlledRobot* getRobot(const std::string& p_robot_name);

    // ------------------------------------------------------------------------
    //! \brief Get the current robot (first loaded robot).
    //! \return Pointer to current robot, nullptr if no robot loaded.
    // ------------------------------------------------------------------------
    RobotManager::ControlledRobot* currentRobot() const;

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
    //! \brief Set robot visibility.
    //! \param p_robot_name Name of the robot.
    //! \param p_visible Visibility flag.
    //! \return true if successful.
    // ------------------------------------------------------------------------
    bool setRobotVisibility(const std::string& p_robot_name, bool p_visible);

    // ------------------------------------------------------------------------
    //! \brief Set robot scale.
    //! \param p_robot_name Name of the robot.
    //! \param p_scale Scale factor.
    //! \return true if successful.
    // ------------------------------------------------------------------------
    bool setRobotScale(const std::string& p_robot_name, float p_scale);

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
    //! \return Map of robot names and visual data.
    // ------------------------------------------------------------------------
    std::unordered_map<std::string, ControlledRobot> const& robots() const
    {
        return m_robots;
    }

    std::unordered_map<std::string, ControlledRobot>& robots()
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

    // ------------------------------------------------------------------------
    //! \brief Update robot link transforms from joint values.
    //! \param p_robot_name Name of the robot.
    //! \return true if successful.
    // ------------------------------------------------------------------------
    bool updateRobotLinkTransforms(const std::string& p_robot_name);

private:

    //! \brief Controlled robots.
    std::unordered_map<std::string, ControlledRobot> m_robots;
    //! \brief Current controlled robot.
    ControlledRobot* m_current_robot = nullptr;
    //! \brief Last error message.
    std::string m_error;
};

} // namespace robotik::renderer
