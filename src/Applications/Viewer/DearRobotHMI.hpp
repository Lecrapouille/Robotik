/**
 * @file DearRobotHMI.hpp
 * @brief ImGui-based HMI for robot control and visualization.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#pragma once

#include "Robotik/Renderer/Camera/OrbitController.hpp"
#include "Robotik/Renderer/Managers/RobotManager.hpp"

#include <functional>
#include <string>
#include <vector>

namespace robotik::application
{

// ****************************************************************************
//! \brief ImGui-based Human-Machine Interface for robot control.
//!
//! This class provides a comprehensive UI for:
//! - Robot management (load/remove robots)
//! - Control mode selection (manual, animation, IK, trajectory)
//! - Joint control with sliders
//! - End effector selection for IK
//! - Camera target selection
// ****************************************************************************
class DearRobotHMI
{
public:

    // ----------------------------------------------------------------------------
    //! \brief Constructor.
    //! \param p_robot_manager Reference to robot manager for robot operations.
    //! \param p_orbit_controller Reference to orbit controller for camera
    //! updates.
    //! \param p_halt_callback Callback to halt the application.
    // ----------------------------------------------------------------------------
    DearRobotHMI(robotik::renderer::RobotManager& p_robot_manager,
                 robotik::renderer::OrbitController& p_orbit_controller,
                 std::function<void()> const& p_halt_callback);

    // ----------------------------------------------------------------------------
    //! \brief Render the main control panel.
    // ----------------------------------------------------------------------------
    void onDrawMainPanel();

    // ----------------------------------------------------------------------------
    //! \brief Render the menu bar.
    // ----------------------------------------------------------------------------
    void onDrawMenuBar();

private:

    // ----------------------------------------------------------------------------
    //! \brief Robot management panel (add/remove/select robots).
    // ----------------------------------------------------------------------------
    void robotManagementPanel();

    // ----------------------------------------------------------------------------
    //! \brief Robot list panel.
    // ----------------------------------------------------------------------------
    void robotListPanel();

    // ----------------------------------------------------------------------------
    //! \brief Load robot panel.
    // ----------------------------------------------------------------------------
    void loadRobotPanel();

    // ----------------------------------------------------------------------------
    //! \brief Control mode selection panel.
    // ----------------------------------------------------------------------------
    void controlModePanel();

    // ----------------------------------------------------------------------------
    //! \brief Joint control panel with sliders.
    // ----------------------------------------------------------------------------
    void jointControlPanel();

    // ----------------------------------------------------------------------------
    //! \brief End effector selection panel for IK.
    // ----------------------------------------------------------------------------
    void endEffectorPanel();

    // ----------------------------------------------------------------------------
    //! \brief Camera target selection panel.
    // ----------------------------------------------------------------------------
    void cameraTargetPanel();

    // ----------------------------------------------------------------------------
    //! \brief Get list of all node names from current robot.
    //! \return Vector of node names.
    // ----------------------------------------------------------------------------
    std::vector<std::string> getNodeNames() const;

    // ----------------------------------------------------------------------------
    //! \brief Get list of end effector names from current robot.
    //! \return Vector of end effector names.
    // ----------------------------------------------------------------------------
    std::vector<std::string> getEndEffectorNames() const;

    // ----------------------------------------------------------------------------
    //! \brief Refresh the list of robots
    // ----------------------------------------------------------------------------
    void refreshRobotList();

    // ----------------------------------------------------------------------------
    //! \brief Refresh the list of nodes and end effectors of the current robot
    // ----------------------------------------------------------------------------
    void refreshCurrentRobotCaches();

    // ----------------------------------------------------------------------------
    //! \brief Set the selected robot
    //! \param p_name The name of the robot to select
    //! \return True if the robot was selected, false otherwise
    // ----------------------------------------------------------------------------
    bool setSelectedRobot(std::string const& p_name);

    // ----------------------------------------------------------------------------
    //! \brief Open file dialog to load a robot from URDF file.
    // ----------------------------------------------------------------------------
    void loadRobot() const;

    // ----------------------------------------------------------------------------
    //! \brief Remove a robot
    //! \param p_name The name of the robot to remove
    // ----------------------------------------------------------------------------
    void removeRobot(std::string const& p_name);

    // ----------------------------------------------------------------------------
    //! \brief Initialize IK and trajectory configurations for a robot.
    //! \param p_robot The robot to initialize.
    // ----------------------------------------------------------------------------
    void initializeRobotConfigurations(
        renderer::RobotManager::ControlledRobot& p_robot);

    // ----------------------------------------------------------------------------
    //! \brief Open about dialog
    // ----------------------------------------------------------------------------
    void about() const;

private:

    //! \brief Reference to robot manager
    robotik::renderer::RobotManager& m_robot_manager;
    //! \brief Reference to orbit controller
    robotik::renderer::OrbitController& m_orbit_controller;
    //! \brief Callback to halt the application
    std::function<void()> m_halt_callback;
    //! \brief Currently selected robot name
    std::string m_selected_robot;
    //! \brief Cache the list of robots
    std::vector<std::string> m_robot_list;
    //! \brief Cache the list of nodes of the current robot
    std::vector<std::string> m_node_names;
    //! \brief Cache the list of end effectors of the current robot
    std::vector<std::string> m_end_effector_names;
    //! \brief Flag to show about dialog
    bool m_show_about = false;
};

} // namespace robotik::application
