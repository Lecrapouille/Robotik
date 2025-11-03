/**
 * @file ApplicationController.hpp
 * @brief Controller for robot control logic (MVC pattern).
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#pragma once

#include "Robotik/Core/Robot/ControlledRobot.hpp"
#include "Robotik/Core/Robot/TeachPendant.hpp"
#include "Robotik/Core/Solvers/IKSolver.hpp"
#include "Robotik/Renderer/Managers/RobotManager.hpp"

#include <memory>
#include <string>
#include <unordered_map>

namespace robotik::application
{

// ****************************************************************************
//! \brief Controller of the Model-View-Controller pattern.
//!
//! This class makes the bridge between the model (robot) and the view
//! (DearImGui) by handling the robot management and teach pendant control.
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
    //! \brief Initialize robot configurations with teach pendant.
    //! \param p_robot_name The robot name.
    //! \param p_robot_blueprint The robot's blueprint.
    //! \param p_control_link Control link name. If empty, uses first end
    //! effector.
    //! \param p_joint_positions Initial joint positions. If empty, uses
    //! neutral.
    //! \param p_camera_target Camera target node name. If empty, uses root.
    //! \return true if successful.
    // ----------------------------------------------------------------------------
    bool initializeRobot(std::string const& p_robot_name,
                         robotik::Blueprint&& p_robot_blueprint,
                         std::string const& p_control_link = "",
                         std::vector<double> const& p_joint_positions = {},
                         std::string const& p_camera_target = "");

    // ----------------------------------------------------------------------------
    //! \brief Update all robots based on their control modes.
    //! \param p_elapsed_time Elapsed time since start in seconds.
    //! \param p_dt Delta time in seconds.
    // ----------------------------------------------------------------------------
    void update(double p_elapsed_time, double p_dt);

    // ----------------------------------------------------------------------------
    //! \brief Set end effector for teach pendant.
    //! \param p_robot_name Robot name.
    //! \param p_link_name Link name.
    //! \return true if successful.
    // ----------------------------------------------------------------------------
    bool setEndEffector(std::string const& p_robot_name,
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
    //! \brief Get controlled robot by name.
    //! \param p_robot_name Robot name.
    //! \return Pointer to controlled robot, nullptr if not found.
    // ----------------------------------------------------------------------------
    ControlledRobot* getControlledRobot(std::string const& p_robot_name);

    // ----------------------------------------------------------------------------
    //! \brief Get the shared teach pendant.
    //! \return Pointer to teach pendant.
    // ----------------------------------------------------------------------------
    robotik::TeachPendant* getTeachPendant();

private:

    //! \brief Reference to robot manager (for basic robots).
    renderer::RobotManager& m_robot_manager;
    //! \brief Controlled robots indexed by robot name.
    std::unordered_map<std::string, std::unique_ptr<ControlledRobot>>
        m_controlled_robots;
    //! \brief Shared teach pendant for all robots.
    std::unique_ptr<robotik::TeachPendant> m_teach_pendant;
    //! \brief Shared IK solver for all robots.
    std::unique_ptr<robotik::IKSolver> m_ik_solver;
};

} // namespace robotik::application
