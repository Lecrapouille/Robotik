/**
 * @file HMI.hpp
 * @brief ImGui-based HMI for robot control and visualization.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#pragma once

#include "Controller.hpp"

#include "Robotik/Renderer/Camera/OrbitController.hpp"
#include "Robotik/Renderer/Managers/RobotManager.hpp"

#include <imgui.h>

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
class HMI
{
public:

    // ----------------------------------------------------------------------------
    //! \brief Constructor.
    //! \param p_robot_manager Reference to robot manager for robot operations.
    //! \param p_robot_controller Reference to robot controller.
    //! \param p_orbit_controller Reference to orbit controller for camera
    //! updates.
    //! \param p_halt_callback Callback to halt the application.
    // ----------------------------------------------------------------------------
    HMI(robotik::renderer::RobotManager& p_robot_manager,
        Controller& p_robot_controller,
        robotik::renderer::OrbitController& p_orbit_controller,
        std::function<void()> const& p_halt_callback);

    // ----------------------------------------------------------------------------
    //! \brief Render the teach pendant control panel.
    // ----------------------------------------------------------------------------
    void onDrawMainPanel();

    // ----------------------------------------------------------------------------
    //! \brief Render the menu bar.
    // ----------------------------------------------------------------------------
    void onDrawMenuBar();

    // ----------------------------------------------------------------------------
    //! \brief Render the robot management dockable window.
    // ----------------------------------------------------------------------------
    void onDrawRobotManagementWindow();

    // ----------------------------------------------------------------------------
    //! \brief Render the camera target dockable window.
    // ----------------------------------------------------------------------------
    void onDrawCameraTargetWindow();

    // ----------------------------------------------------------------------------
    //! \brief Get the currently selected robot name
    //! \return The name of the selected robot, empty string if none selected
    // ----------------------------------------------------------------------------
    std::string const& selectedRobot() const
    {
        return m_selected_robot;
    }

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
    //! \brief Draw a robot list item with context menu.
    //! \param p_robot_name Name of the robot.
    // ----------------------------------------------------------------------------
    void drawRobotListItem(std::string const& p_robot_name);

    // ----------------------------------------------------------------------------
    //! \brief Draw context menu for a robot.
    //! \param p_robot_name Name of the robot.
    // ----------------------------------------------------------------------------
    void drawRobotListContextMenu(std::string const& p_robot_name);

    // ----------------------------------------------------------------------------
    //! \brief Load robot panel.
    // ----------------------------------------------------------------------------
    void loadRobotPanel();

    // ----------------------------------------------------------------------------
    //! \brief Camera target selection panel.
    // ----------------------------------------------------------------------------
    void cameraTargetPanel();

    // ----------------------------------------------------------------------------
    //! \brief Draw camera target combo box.
    //! \param p_current_camera_target Current camera target name.
    // ----------------------------------------------------------------------------
    void
    drawCameraTargetCombo(std::string const& p_current_camera_target) const;

    // ----------------------------------------------------------------------------
    //! \brief Draw camera tracking checkbox.
    //! \param p_robot Controlled robot.
    // ----------------------------------------------------------------------------
    void drawCameraTrackingCheckbox(ControlledRobot* p_robot) const;

    // ----------------------------------------------------------------------------
    //! \brief Teach pendant control panel with tabs.
    // ----------------------------------------------------------------------------
    void teachPendantPanel();

    // ----------------------------------------------------------------------------
    //! \brief Draw control mode tabs (notebook) and return selected mode.
    //! \param p_current_mode Current control mode.
    //! \return Selected control mode.
    // ----------------------------------------------------------------------------
    ControlledRobot::ControlMode
    drawControlModeTabs(ControlledRobot::ControlMode p_current_mode) const;

    // ----------------------------------------------------------------------------
    //! \brief Draw joint control section.
    //! \param p_robot Controlled robot.
    //! \param p_teach_pendant Teach pendant instance.
    // ----------------------------------------------------------------------------
    void drawJointControlSection(ControlledRobot* p_robot,
                                 robotik::TeachPendant* p_teach_pendant) const;

    // ----------------------------------------------------------------------------
    //! \brief Draw cartesian control section with horizontal layout.
    //! \param p_robot Controlled robot.
    //! \param p_teach_pendant Teach pendant instance.
    // ----------------------------------------------------------------------------
    void drawCartesianControlSection(ControlledRobot* p_robot,
                                     robotik::TeachPendant* p_teach_pendant);

    // ----------------------------------------------------------------------------
    //! \brief Draw frame selection combo.
    // ----------------------------------------------------------------------------
    void drawFrameSelection() const;

    // ----------------------------------------------------------------------------
    //! \brief Draw translation controls.
    //! \param p_teach_pendant Teach pendant instance.
    // ----------------------------------------------------------------------------
    void drawTranslationControls(robotik::TeachPendant* p_teach_pendant) const;

    // ----------------------------------------------------------------------------
    //! \brief Draw rotation controls.
    //! \param p_teach_pendant Teach pendant instance.
    // ----------------------------------------------------------------------------
    void drawRotationControls(robotik::TeachPendant* p_teach_pendant) const;

    // ----------------------------------------------------------------------------
    //! \brief Draw waypoints section.
    //! \param p_robot Controlled robot.
    //! \param p_teach_pendant Teach pendant instance.
    // ----------------------------------------------------------------------------
    void drawWaypointsSection(ControlledRobot const* p_robot,
                              robotik::TeachPendant* p_teach_pendant) const;

    // ----------------------------------------------------------------------------
    //! \brief Draw trajectory playback section.
    //! \param p_robot Controlled robot.
    //! \param p_teach_pendant Teach pendant instance.
    // ----------------------------------------------------------------------------
    void
    drawTrajectoryPlaybackSection(ControlledRobot* p_robot,
                                  robotik::TeachPendant* p_teach_pendant) const;

    // ----------------------------------------------------------------------------
    //! \brief End effector selection panel for teach pendant.
    // ----------------------------------------------------------------------------
    void endEffectorSelectionPanel();

    // ----------------------------------------------------------------------------
    //! \brief Render a node in the end effector combo box.
    //! \param p_node_name Name of the node.
    //! \param p_color Color for the node.
    //! \param p_current_end_effector Current end effector name.
    //! \param p_is_header Whether this is a header item.
    // ----------------------------------------------------------------------------
    void renderEndEffectorNode(std::string const& p_node_name,
                               ImVec4 p_color,
                               std::string const& p_current_end_effector,
                               bool p_is_header = false);

    // ----------------------------------------------------------------------------
    //! \brief Draw end effector combo box items.
    //! \param p_end_effectors List of end effector names.
    //! \param p_current_end_effector Current end effector name.
    // ----------------------------------------------------------------------------
    void drawEndEffectorCombo(std::vector<std::string> const& p_end_effectors,
                              std::string const& p_current_end_effector);

    // ----------------------------------------------------------------------------
    //! \brief Draw all nodes combo box items.
    //! \param p_nodes List of all node names.
    //! \param p_end_effectors List of end effector names.
    //! \param p_current_end_effector Current end effector name.
    // ----------------------------------------------------------------------------
    void drawAllNodesCombo(std::vector<std::string> const& p_nodes,
                           std::vector<std::string> const& p_end_effectors,
                           std::string const& p_current_end_effector);

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
    //! \brief Open about dialog
    // ----------------------------------------------------------------------------
    void about() const;

private:

    //! \brief Reference to robot manager
    robotik::renderer::RobotManager& m_robot_manager;
    //! \brief Reference to Model-View-Controller controller
    Controller& m_controller;
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
