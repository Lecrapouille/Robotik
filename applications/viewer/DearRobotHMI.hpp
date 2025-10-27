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

namespace robotik::renderer
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
    DearRobotHMI(RobotManager& p_robot_manager,
                 OrbitController& p_orbit_controller,
                 std::function<void()> p_halt_callback);

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
    //! \brief Get list of joints from current robot.
    //! \return Vector of pairs (joint name, joint position).
    // ----------------------------------------------------------------------------
    std::vector<std::pair<std::string, double>> getJoints() const;

private:

    //! \brief Reference to robot manager
    RobotManager& m_robot_manager;
    //! \brief Reference to orbit controller
    OrbitController& m_orbit_controller;
    //! \brief Callback to halt the application
    std::function<void()> m_halt_callback;

    //! \brief Currently selected robot name
    std::string m_selected_robot;
    //! \brief Buffer for URDF file path input
    char m_urdf_path_buffer[256] = { 0 };
};

} // namespace robotik::renderer
