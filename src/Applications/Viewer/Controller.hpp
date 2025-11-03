/**
 * @file ApplicationController.hpp
 * @brief Controller for robot control logic (MVC pattern).
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#pragma once

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
public:

    // ----------------------------------------------------------------------------
    //! \brief Constructor.
    //! \param p_robot_manager Reference to robot manager.
    // ----------------------------------------------------------------------------
    explicit Controller(renderer::RobotManager& p_robot_manager);

    // ----------------------------------------------------------------------------
    //! \brief Initialize all robots.
    //! \param p_link_name Optional control link name. If empty, uses
    //! first end effector.
    //! \return true if successful.
    // ----------------------------------------------------------------------------
    bool initializeRobots(std::string const& p_link_name);

    // ----------------------------------------------------------------------------
    //! \brief Initialize robot configurations (IK targets, trajectory configs).
    //! \param p_robot The robot to initialize.
    //! \param p_link_name Optional control link name. If empty, uses
    //! first end effector.
    //! \param p_initial_joint_positions Optional initial joint positions. If
    //! empty, uses neutral position.
    //! \return true if successful.
    // ----------------------------------------------------------------------------
    bool initializeRobot(renderer::RobotManager::ControlledRobot& p_robot,
                         std::string const& p_link_name = "",
                         JointValues const& p_initial_joint_positions = {});

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
    //! \brief Render IK failure popup with recovery options.
    //! \param p_robot_name Robot name.
    // ----------------------------------------------------------------------------
    void renderIKFailurePopup(std::string const& p_robot_name);

private:

    // ----------------------------------------------------------------------------
    //! \brief Robot state for tracking IK target indices.
    // ----------------------------------------------------------------------------
    struct RobotState
    {
        size_t target_pose_index = 0;
        bool solution_computed = false;
        JointValues ik_solution;
        JointValues home_configuration;
        bool ik_failed = false;
        bool show_failure_popup = false;
        bool return_to_home_requested = false;
        bool trajectory_from_ik =
            false; // Flag to indicate trajectory was created by IK
        bool ik_cycle_complete = false; // Flag to indicate all targets reached
    };

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
                         double p_time) const;

    // ----------------------------------------------------------------------------
    //! \brief Handle inverse kinematics mode.
    //! \param p_robot The robot.
    //! \param p_dt Delta time.
    //! \param p_robot_state Robot state for tracking.
    // ----------------------------------------------------------------------------
    void
    handleInverseKinematics(renderer::RobotManager::ControlledRobot& p_robot,
                            double p_dt,
                            RobotState& p_robot_state) const;

    // ----------------------------------------------------------------------------
    //! \brief Handle trajectory mode.
    //! \param p_robot The robot.
    //! \param p_time Elapsed time.
    //! \param p_dt Delta time.
    // ----------------------------------------------------------------------------
    void handleTrajectory(renderer::RobotManager::ControlledRobot& p_robot,
                          double p_time,
                          double p_dt) const;

private:

    //! \brief Reference to robot manager.
    renderer::RobotManager& m_robot_manager;
    //! \brief Robot states indexed by robot name (mutable for state tracking in
    //! const methods).
    mutable std::unordered_map<std::string, RobotState> m_robot_states;
};

} // namespace robotik::application
