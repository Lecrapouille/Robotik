/**
 * @file ApplicationController.hpp
 * @brief Controller for robot control logic (MVC pattern).
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#pragma once

#include "Robotik/Core/Robot/Robot.hpp"
#include "Robotik/Core/Robot/TeachPendant.hpp"
#include "Robotik/Renderer/Managers/RobotManager.hpp"

#include <memory>
#include <string>
#include <unordered_map>

namespace robotik::application
{

// ****************************************************************************
//! \brief Controlled robot class that extends Robot with control capabilities.
//!
//! This class inherits from Robot and adds teach pendant functionality for
//! interactive robot control, as well as camera tracking support.
// ****************************************************************************
class ControlledRobot: public robotik::Robot
{
public:

    // ------------------------------------------------------------------------
    //! \brief Constructor forwarding to Robot constructor.
    //! \param p_name The name of the robot.
    //! \param p_blueprint The robot's kinematic structure.
    // ------------------------------------------------------------------------
    ControlledRobot(std::string const& p_name, robotik::Blueprint&& p_blueprint)
        : Robot(p_name, std::move(p_blueprint))
    {
    }

    //! \brief Teach pendant for interactive robot control
    std::unique_ptr<robotik::TeachPendant> teach_pendant;
    //! \brief Tracked node for camera
    robotik::Node const* camera_target = nullptr;
    //! \brief Camera tracking enabled
    bool camera_tracking_enabled = true;
};

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
    //! \brief Get teach pendant for a robot.
    //! \param p_robot_name Robot name.
    //! \return Pointer to teach pendant, nullptr if not found.
    // ----------------------------------------------------------------------------
    robotik::TeachPendant* getTeachPendant(std::string const& p_robot_name);

private:

    //! \brief Reference to robot manager (for basic robots).
    renderer::RobotManager& m_robot_manager;
    //! \brief Controlled robots indexed by robot name.
    std::unordered_map<std::string, std::unique_ptr<ControlledRobot>>
        m_controlled_robots;
};

} // namespace robotik::application
