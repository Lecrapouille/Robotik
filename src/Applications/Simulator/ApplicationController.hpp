/**
 * @file ApplicationController.hpp
 * @brief Application control logic (Controller module of the MVC pattern).
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#pragma once

#include "CameraController.hpp"
#include "Configuration.hpp"
#include "ControlledRobot.hpp"

#include "Robotik/Core/Managers/BehaviorTreeManager.hpp"
#include "Robotik/Core/Managers/RobotManager.hpp"
#include "Robotik/Core/Managers/WaypointManager.hpp"
#include "Robotik/Core/Robot/TeachPendant.hpp"
#include "Robotik/Core/Solvers/IKSolver.hpp"
#include "Robotik/Core/Solvers/TrajectoryController.hpp"

#include <memory>
#include <string>

namespace robotik::application
{

// ****************************************************************************
//! \brief Application controller of the Model-View-Controller pattern.
//!
//! This class makes the bridge between the model (robot, camera) and the view
//! (DearImGui) by handling the robot management and teach pendant control.
// ****************************************************************************
class ApplicationController
{
public:

    // ----------------------------------------------------------------------------
    //! \brief Constructor.
    //! \param p_config Application configuration.
    // ----------------------------------------------------------------------------
    explicit ApplicationController(Configuration const& p_config);

    // ----------------------------------------------------------------------------
    //! \brief Load a robot from a URDF file.
    //! \param p_urdf_file The path to the URDF file.
    //! \return Pointer to the loaded robot, nullptr if failed. You can get the
    //! error message from the error() method.
    // ----------------------------------------------------------------------------
    ControlledRobot* loadRobot(std::string const& p_urdf_file);

    // ----------------------------------------------------------------------------
    //! \brief Initialize robot that has been just been created (from URDF
    //! file) and stored in the robot manager. Set initial joint positions, home
    //! position, end effector and camera target.
    //!
    //! \param p_robot The robot to initialize.
    //! \param p_control_link The link name to control (usually the end
    //! effector). If empty, uses first end effector.
    //! \param p_joint_positions Initial joint positions. If empty, uses
    //! neutral.
    //! \param p_camera_target The node name to track with the camera. If empty,
    //! uses the base link (root node).
    //! \return True if successful, false otherwise (e.g. link not found). You
    //! can get the error message from the error() method.
    // ----------------------------------------------------------------------------
    bool initializeRobot(ControlledRobot& p_robot,
                         std::string const& p_control_link = "",
                         std::vector<double> const& p_joint_positions = {},
                         std::string const& p_camera_target = "") const;

    // ----------------------------------------------------------------------------
    //! \brief Update all robots based on their control modes.
    //! \param p_dt Delta time in seconds.
    // ----------------------------------------------------------------------------
    void update(float const p_dt);

    // ----------------------------------------------------------------------------
    //! \brief Set the end effector to control.
    //! \param p_robot_name Robot name.
    //! \param p_link_name The link name to control (usually the end effector).
    //! \return True if successful (recognized link name), false otherwise (e.g.
    //! link not found). You can get the error message from the error() method.
    // ----------------------------------------------------------------------------
    bool setEndEffector(std::string const& p_robot_name,
                        std::string const& p_link_name) const;

    // ----------------------------------------------------------------------------
    //! \brief Set the camera target to track a joint or link of the robot.
    //! \param p_robot_name Robot name.
    //! \param p_node_name The node name to track with the camera. If empty,
    //! uses the base link (root node).
    //! \return True if successful, false otherwise (e.g. node not found). You
    //! can get the error message from the error() method.
    // ----------------------------------------------------------------------------
    bool setCameraTarget(std::string const& p_robot_name,
                         std::string const& p_node_name) const;

    // ----------------------------------------------------------------------------
    //! \brief Initialize behavior tree with robot actions.
    //! This registers all robot-specific actions in the behavior tree factory.
    //! Should be called after loading a robot.
    //! \param p_robot The robot to use for behavior tree actions.
    // ----------------------------------------------------------------------------
    void initializeBehaviorTree(ControlledRobot& p_robot);

    // ----------------------------------------------------------------------------
    //! \brief Set the Cartesian frame to use for the Cartesian control.
    //! \param p_robot_name Robot name.
    //! \param p_node_name The node name to use as the Cartesian frame. If
    //! empty, uses the base link (root node).
    //! \return True if successful, false otherwise (e.g. node not found). You
    //! can get the error message from the error() method.
    // ----------------------------------------------------------------------------
    bool setCartesianFrame(std::string const& p_robot_name,
                           std::string const& p_node_name) const;

    // ----------------------------------------------------------------------------
    //! \brief Get the pointer to the current robot.
    //! \return Pointer to the current robot, nullptr if no robot loaded.
    // ----------------------------------------------------------------------------
    ControlledRobot* getCurrentRobot() const;

    // ----------------------------------------------------------------------------
    //! \brief Find the robot by its name and return a pointer to it.
    //! \param p_robot_name The name of the robot to find.
    //! \return Pointer to the robot, nullptr if not found.
    // ----------------------------------------------------------------------------
    ControlledRobot* getRobot(std::string const& p_robot_name) const;

    // ----------------------------------------------------------------------------
    //! \brief Get the reference to the teach pendant used to control the
    //! robots.
    //! \return Reference to the teach pendant.
    // ----------------------------------------------------------------------------
    robotik::TeachPendant& getTeachPendant()
    {
        return *m_teach_pendant;
    }

    // ----------------------------------------------------------------------------
    //! \brief Get the reference to the IK solver used to solve the inverse
    //! kinematics problem.
    //! \return Reference to the IK solver.
    // ----------------------------------------------------------------------------
    robotik::IKSolver& getIKSolver()
    {
        return *m_ik_solver;
    }

    // ----------------------------------------------------------------------------
    //! \brief Get the reference to the camera controller used to control the
    //! camera.
    //! \return Reference to the camera controller.
    // ----------------------------------------------------------------------------
    CameraController& getCameraController()
    {
        return *m_camera_controller;
    }

    // ----------------------------------------------------------------------------
    //! \brief Get the reference to the waypoint manager used to manage the
    //! waypoints.
    //! \return Reference to the waypoint manager.
    // ----------------------------------------------------------------------------
    WaypointManager& getWaypointManager()
    {
        return *m_waypoint_manager;
    }

    // ----------------------------------------------------------------------------
    //! \brief Get the reference to the trajectory controller used to manage the
    //! trajectories.
    //! \return Reference to the trajectory controller.
    // ----------------------------------------------------------------------------
    TrajectoryController& getTrajectoryController()
    {
        return *m_trajectory_controller;
    }

    // ----------------------------------------------------------------------------
    //! \brief Get the reference to the robot manager used to store robots
    //! instances.
    //! \return Reference to the robot manager.
    // ----------------------------------------------------------------------------
    RobotManager& getRobotManager()
    {
        return *m_robot_manager;
    }

    // ----------------------------------------------------------------------------
    //! \brief Get the reference to the behavior tree manager used to manage
    //! behavior trees.
    //! \return Reference to the behavior tree manager.
    // ----------------------------------------------------------------------------
    BehaviorTreeManager& getBehaviorTreeManager()
    {
        return *m_behavior_tree_manager;
    }

    // ----------------------------------------------------------------------------
    //! \brief Get the error message.
    //! \return Error message.
    // ----------------------------------------------------------------------------
    std::string const& error() const
    {
        return m_error;
    }

private:

    //! \brief Application configuration.
    Configuration const& m_config;
    //! \brief Robot manager used to store robots instances.
    std::unique_ptr<RobotManager> m_robot_manager;
    //! \brief Teach pendant used to control the robots.
    std::unique_ptr<TeachPendant> m_teach_pendant;
    //! \brief IK solver used to solve the inverse kinematics problem.
    std::unique_ptr<IKSolver> m_ik_solver;
    //! \brief Camera controller used to control the camera.
    std::unique_ptr<CameraController> m_camera_controller;
    //! \brief Waypoint manager used to manage the waypoints.
    std::unique_ptr<WaypointManager> m_waypoint_manager;
    //! \brief Trajectory controller used to manage the trajectories.
    std::unique_ptr<TrajectoryController> m_trajectory_controller;
    //! \brief Behavior tree manager used to manage behavior trees.
    std::unique_ptr<BehaviorTreeManager> m_behavior_tree_manager;
    //! \brief Error message.
    std::string m_error;
};

} // namespace robotik::application
