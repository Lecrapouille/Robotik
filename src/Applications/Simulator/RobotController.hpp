/**
 * @file RobotController.hpp
 * @brief Robot controller for robot control logic (MVC pattern).
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#pragma once

#include "CameraViewModel.hpp"
#include "Configuration.hpp"
#include "ControlledRobot.hpp"

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
//! \brief Robot controller of the Model-View-Controller pattern.
//!
//! This class makes the bridge between the model (robot) and the view
//! (DearImGui) by handling the robot management and teach pendant control.
// ****************************************************************************
class RobotController
{
public:

    // ----------------------------------------------------------------------------
    //! \brief Constructor.
    //! \param p_robot_manager Reference to robot manager.
    // ----------------------------------------------------------------------------
    explicit RobotController(Configuration const& p_config);

    // ----------------------------------------------------------------------------
    //! \brief Initialize robot configurations with teach pendant that has been
    //! just been created (from URDF file).
    //! \param p_robot The robot to initialize.
    //! \param p_control_link Control link name. If empty, uses first end
    //! effector.
    //! \param p_joint_positions Initial joint positions. If empty, uses
    //! neutral.
    //! \param p_camera_target Camera target node name. If empty, uses root.
    //! \return true if successful.
    // ----------------------------------------------------------------------------
    bool initializeRobot(ControlledRobot& p_robot,
                         std::string const& p_control_link = "",
                         std::vector<double> const& p_joint_positions = {},
                         std::string const& p_camera_target = "") const;

    // ----------------------------------------------------------------------------
    //! \brief Update all robots based on their control modes.
    //! \param p_dt Delta time in seconds.
    // ----------------------------------------------------------------------------
    void update(double p_dt);

    // ----------------------------------------------------------------------------
    //! \brief Set end effector (Root link of the robot) to control.
    //! \param p_robot_name Robot name.
    //! \param p_link_name Link name.
    //! \return true if successful (recognized link name).
    // ----------------------------------------------------------------------------
    bool setEndEffector(std::string const& p_robot_name,
                        std::string const& p_link_name);

    // ----------------------------------------------------------------------------
    //! \brief Set camera target for tracking a joint or link of the robot.
    //! \param p_robot_name Robot name.
    //! \param p_node_name Node name.
    //! \return true if successful.
    // ----------------------------------------------------------------------------
    bool setCameraTarget(std::string const& p_robot_name,
                         std::string const& p_node_name);

    // ----------------------------------------------------------------------------
    //! \brief Set Cartesian frame for Cartesian control.
    //! \param p_robot_name Robot name.
    //! \param p_node_name Node name (empty string = world frame).
    //! \return true if successful.
    // ----------------------------------------------------------------------------
    bool setCartesianFrame(std::string const& p_robot_name,
                           std::string const& p_node_name);

    // ----------------------------------------------------------------------------
    //! \brief Get the current robot.
    //! \return Pointer to current robot, nullptr if no robot loaded.
    // ----------------------------------------------------------------------------
    ControlledRobot* getCurrentRobot() const;

    // ----------------------------------------------------------------------------
    //! \brief Get the robot by name.
    //! \param p_robot_name Robot name.
    //! \return Pointer to controlled robot, nullptr if not found.
    // ----------------------------------------------------------------------------
    ControlledRobot* getRobot(std::string const& p_robot_name) const;

    // ----------------------------------------------------------------------------
    //! \brief Get the shared teach pendant.
    //! \return Pointer to teach pendant.
    // ----------------------------------------------------------------------------
    robotik::TeachPendant& getTeachPendant()
    {
        return *m_teach_pendant;
    }

    // ----------------------------------------------------------------------------
    //! \brief Get the shared IK solver.
    //! \return Pointer to IK solver.
    // ----------------------------------------------------------------------------
    robotik::IKSolver& getIKSolver()
    {
        return *m_ik_solver;
    }

    // ----------------------------------------------------------------------------
    //! \brief Get the reference to camera model.
    //! \return Reference to camera model.
    // ----------------------------------------------------------------------------
    CameraViewModel& getCameraModel()
    {
        return *camera_model;
    }

    // ----------------------------------------------------------------------------
    //! \brief Get the reference to waypoint manager.
    //! \return Reference to waypoint manager.
    // ----------------------------------------------------------------------------
    WaypointManager& getWaypointManager()
    {
        return *waypoint_manager;
    }

    // ----------------------------------------------------------------------------
    //! \brief Get the reference to trajectory controller.
    //! \return Reference to trajectory controller.
    // ----------------------------------------------------------------------------
    TrajectoryController& getTrajectoryController()
    {
        return *trajectory_controller;
    }

    // ----------------------------------------------------------------------------
    //! \brief Get the reference to robot manager.
    //! \return Reference to robot manager.
    // ----------------------------------------------------------------------------
    RobotManager& getRobotManager()
    {
        return *m_robot_manager;
    }

private:

    //! \brief Reference to robot manager (for basic robots).
    std::unique_ptr<RobotManager> m_robot_manager;
    //! \brief Shared teach pendant for all robots.
    std::unique_ptr<robotik::TeachPendant> m_teach_pendant;
    //! \brief Shared IK solver for all robots.
    std::unique_ptr<robotik::IKSolver> m_ik_solver;
    //! \brief Reference to camera model.
    std::unique_ptr<CameraViewModel> camera_model;
    //! \brief Reference to waypoint manager.
    std::unique_ptr<WaypointManager> waypoint_manager;
    //! \brief Reference to trajectory controller.
    std::unique_ptr<TrajectoryController> trajectory_controller;
    //! \brief Cache list of robot names.
    std::vector<std::string> m_robot_list;
};

} // namespace robotik::application
