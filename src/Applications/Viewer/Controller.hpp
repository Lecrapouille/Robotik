/**
 * @file ApplicationController.hpp
 * @brief Controller for robot control logic (MVC pattern).
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#pragma once

#include "Configuration.hpp"
#include "Robotik/Renderer/Managers/RobotManager.hpp"

#include <string>
#include <unordered_map>

namespace robotik::application
{

// ****************************************************************************
//! \brief Controller of the Model-View-Controller pattern.
//!
//! This class makes the bridge between the model (robot) and the view
//! (DearImGui) by handling the robot management and control mode selection.
// ****************************************************************************
class Controller
{
    // ----------------------------------------------------------------------------
    //! \brief Robot state for tracking IK target indices.
    // ----------------------------------------------------------------------------
    struct RobotState
    {
        size_t target_pose_index = 0;
    };

public:

    // ----------------------------------------------------------------------------
    //! \brief Constructor.
    //! \param p_robot_manager Reference to robot manager.
    // ----------------------------------------------------------------------------
    explicit Controller(renderer::RobotManager& p_robot_manager);

    // ----------------------------------------------------------------------------
    //! \brief Initialize all robots.
    //! \param p_config Configuration.
    //! first end effector.
    //! \return true if successful.
    // ----------------------------------------------------------------------------
    bool setupRobots(Configuration const& p_config);

    // ----------------------------------------------------------------------------
    //! \brief Update all robots based on their control modes.
    //! \param p_elapsed_time Elapsed time since start in seconds.
    //! \param p_dt Delta time in seconds.
    // ----------------------------------------------------------------------------
    void update(double p_elapsed_time, double p_dt);

    // ----------------------------------------------------------------------------
    //! \brief Set control mode for a robot.
    //! \param p_robot_name Robot name.
    //! \param p_mode Control mode.
    //! \return true if successful.
    // ----------------------------------------------------------------------------
    bool setControlMode(std::string const& p_robot_name,
                        renderer::RobotManager::ControlMode p_mode);

    // ----------------------------------------------------------------------------
    //! \brief Set control link for a robot.
    //! \param p_robot_name Robot name.
    //! \param p_link_name Joint name.
    //! \return true if successful.
    // ----------------------------------------------------------------------------
    bool setControlJoint(std::string const& p_robot_name,
                         std::string const& p_link_name);

    // ----------------------------------------------------------------------------
    //! \brief Set camera target for a robot.
    //! \param p_robot_name Robot name.
    //! \param p_node_name Node name.
    //! \return true if successful.
    // ----------------------------------------------------------------------------
    bool setCameraTarget(std::string const& p_robot_name,
                         std::string const& p_node_name);

    // ----------------------------------------------------------------------------
    //! \brief Set camera target for a robot.
    //! \param p_robot The robot.
    //! \param p_node_name Node name.
    //! \param p_use_root_if_not_found Whether to use root if not found.
    //! \return true if successful.
    // ----------------------------------------------------------------------------
    bool setCameraTarget(renderer::RobotManager::ControlledRobot& p_robot,
                         std::string const& p_node_name,
                         bool p_use_root_if_not_found) const;

    // ----------------------------------------------------------------------------
    //! \brief Get error message.
    // ----------------------------------------------------------------------------
    std::string const& error() const
    {
        return m_error;
    }

private:

    // ----------------------------------------------------------------------------
    //! \brief Initialize robot configurations (IK targets, trajectory configs).
    //! \param p_robot The robot to initialize.
    //! \param p_config Configuration.
    //! \return true if successful.
    // ----------------------------------------------------------------------------
    bool initializeRobot(renderer::RobotManager::ControlledRobot& p_robot,
                         Configuration const& p_config);

    // ----------------------------------------------------------------------------
    //! \brief Compute IK target poses for a robot.
    //! \param p_robot The robot.
    // ----------------------------------------------------------------------------
    void computeIKTargetPoses(renderer::RobotManager::ControlledRobot& p_robot);

    // ----------------------------------------------------------------------------
    //! \brief Compute trajectory configurations for a robot.
    //! \param p_robot The robot.
    // ----------------------------------------------------------------------------
    void
    computeTrajectoryConfigs(renderer::RobotManager::ControlledRobot& p_robot);

    // ----------------------------------------------------------------------------
    //! \brief Handle animation mode.
    //! \param p_robot The robot.
    //! \param p_time Elapsed time.
    // ----------------------------------------------------------------------------
    void handleAnimation(renderer::RobotManager::ControlledRobot& p_robot,
                         double p_time);

    // ----------------------------------------------------------------------------
    //! \brief Handle inverse kinematics mode.
    //! \param p_robot The robot.
    //! \param p_dt Delta time.
    //! \param p_robot_state Robot state for tracking.
    // ----------------------------------------------------------------------------
    void
    handleInverseKinematics(renderer::RobotManager::ControlledRobot& p_robot,
                            double p_dt,
                            RobotState& p_robot_state);

    // ----------------------------------------------------------------------------
    //! \brief Handle trajectory mode.
    //! \param p_robot The robot.
    //! \param p_time Elapsed time.
    //! \param p_dt Delta time.
    // ----------------------------------------------------------------------------
    void handleTrajectory(renderer::RobotManager::ControlledRobot& p_robot,
                          double p_time,
                          double p_dt);

private:

    //! \brief Reference to robot manager.
    renderer::RobotManager& m_robot_manager;
    //! \brief Robot states indexed by robot name.
    std::unordered_map<std::string, RobotState> m_robot_states;
    //! \brief Error message.
    std::string m_error;
};

} // namespace robotik::application
