/**
 * @file HMI.cpp
 * @brief ImGui-based HMI for robot control and visualization implementation.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#include "HMI.hpp"
#include "Application.hpp"

#include "ImGuiFileDialog/ImGuiFileDialog.h"
#include "Robotik/Core/Exporters/RobotExporterFactory.hpp"
#include "Robotik/Core/Robot/Blueprint/Node.hpp"
#include "Robotik/Core/Robot/TeachPendant.hpp"
#include "project_info.hpp"

#include <algorithm>
#include <imgui.h>
#include <imgui_stdlib.h>
#include <iostream>

namespace robotik::application
{

// ----------------------------------------------------------------------------
HMI::HMI(robotik::renderer::RobotManager& p_robot_manager,
         Controller& p_robot_controller,
         Application& p_application,
         std::function<void()> const& p_halt_callback)
    : m_robot_manager(p_robot_manager),
      m_controller(p_robot_controller),
      m_application(p_application),
      m_halt_callback(p_halt_callback)
{
    if (auto const* robot = m_robot_manager.currentRobot(); robot != nullptr)
    {
        m_selected_robot = robot->name();
    }
    refreshRobotList();
    refreshCurrentRobotCaches();
}

// ----------------------------------------------------------------------------
void HMI::onDrawMenuBar()
{
    if (ImGui::BeginMenu("File"))
    {
        if (ImGui::MenuItem("Load Robot"))
        {
            loadRobot();
        }
        if (ImGui::MenuItem(
                "Export Robot", nullptr, false, !m_selected_robot.empty()))
        {
            exportRobot();
        }
        if (ImGui::MenuItem("Quit") && m_halt_callback)
        {
            m_halt_callback();
        }
        ImGui::EndMenu();
    }
    if (ImGui::BeginMenu("Help"))
    {
        if (ImGui::MenuItem("About"))
        {
            m_show_about = true;
        }
        ImGui::EndMenu();
    }
    if (m_show_about)
    {
        ImGui::OpenPopup("About Robotik");
        m_show_about = false;
    }
    about();
}

// ----------------------------------------------------------------------------
void HMI::onDrawMainPanel()
{
    ImGui::Begin("Teach Pendant Control");

    if (!m_selected_robot.empty())
    {
        endEffectorSelectionPanel();
        ImGui::Separator();
        teachPendantPanel();
    }
    else
    {
        ImGui::TextColored(ImVec4(1.0f, 1.0f, 0.0f, 1.0f),
                           "No robot selected. Load a robot first.");
    }

    ImGui::End();
}

// ----------------------------------------------------------------------------
void HMI::onDrawRobotManagementWindow()
{
    ImGui::Begin("Robot Management");
    robotManagementPanel();
    ImGui::End();
}

// ----------------------------------------------------------------------------
void HMI::onDrawCameraTargetWindow()
{
    ImGui::Begin("Camera");

    // Camera views (Orbit + predefined views)
    drawCameraControllerPanel();

    ImGui::Separator();

    // Camera target selection (only if robot is selected)
    if (!m_selected_robot.empty())
    {
        cameraTargetPanel();
    }
    else
    {
        ImGui::TextColored(ImVec4(1.0f, 1.0f, 0.0f, 1.0f),
                           "Select a robot to configure camera target");
    }

    ImGui::End();
}

// ----------------------------------------------------------------------------
void HMI::onDrawTrajectoryWindow()
{
    ImGui::Begin("Trajectory");

    if (!m_selected_robot.empty())
    {
        auto* robot = m_controller.getControlledRobot(m_selected_robot);
        if (robot != nullptr)
        {
            auto* teach_pendant = m_controller.getTeachPendant();
            if (teach_pendant != nullptr)
            {
                auto* ik_solver = m_controller.getIKSolver();
                // Configure the teach pendant for this robot
                teach_pendant->setRobot(*robot);
                if (ik_solver != nullptr)
                {
                    teach_pendant->setIKSolver(ik_solver);
                }
                if (robot->end_effector != nullptr)
                {
                    teach_pendant->setEndEffector(*robot->end_effector);
                }

                // Waypoints Section
                drawWaypointsSection(robot, teach_pendant);
                ImGui::Spacing();

                // Trajectory Playback Section
                drawTrajectoryPlaybackSection(robot, teach_pendant);
            }
        }
    }
    else
    {
        ImGui::TextColored(ImVec4(1.0f, 1.0f, 0.0f, 1.0f),
                           "Select a robot to configure trajectory");
    }

    ImGui::End();
}

// ----------------------------------------------------------------------------
void HMI::robotManagementPanel()
{
    if (ImGui::Button("Load Robot"))
    {
        loadRobot();
    }

    robotListPanel();
    loadRobotPanel();
    exportRobotPanel();
}

// ----------------------------------------------------------------------------
void HMI::robotListPanel()
{
    ImGui::Text("Loaded Robots (%zu):", m_robot_list.size());
    ImGui::BeginChild("RobotList", ImVec2(0, 100), true);
    for (const auto& robot_name : m_robot_list)
    {
        drawRobotListItem(robot_name);
    }
    ImGui::EndChild();
}

// ----------------------------------------------------------------------------
void HMI::drawRobotListItem(std::string const& p_robot_name)
{
    if (ImGui::Selectable(p_robot_name.c_str(),
                          m_selected_robot == p_robot_name))
    {
        setSelectedRobot(p_robot_name);
    }

    if (ImGui::BeginPopupContextItem())
    {
        drawRobotListContextMenu(p_robot_name);
        ImGui::EndPopup();
    }
}

// ----------------------------------------------------------------------------
void HMI::drawRobotListContextMenu(std::string const& p_robot_name)
{
    if (ImGui::MenuItem("Load robot"))
    {
        loadRobot();
    }

    if (ImGui::MenuItem("Remove robot"))
    {
        removeRobot(p_robot_name);
    }

    if (auto* controlled = m_controller.getControlledRobot(p_robot_name))
    {
        if (ImGui::MenuItem("Toggle Visibility"))
        {
            controlled->blueprint().enable(!controlled->blueprint().enabled());
        }
    }
}

// ----------------------------------------------------------------------------
void HMI::loadRobotPanel()
{
    if (!ImGuiFileDialog::Instance()->Display("RobotURDFDlg"))
        return;

    if (ImGuiFileDialog::Instance()->IsOk())
    {
        std::string urdf = ImGuiFileDialog::Instance()->GetFilePathName();
        auto* robot = m_robot_manager.loadRobot<ControlledRobot>(urdf);
        if (robot != nullptr)
        {
            std::cout << "✅ Loaded robot from: " << urdf << std::endl;

            // Extract blueprint and initialize controlled robot
            std::string robot_name = robot->name();
            robotik::Blueprint blueprint = std::move(robot->blueprint());
            m_controller.initializeRobot(robot, "", {}, "");

            setSelectedRobot(robot_name);
            refreshRobotList();
        }
        else
        {
            std::cerr << "❌ Failed to load robot: " << m_robot_manager.error()
                      << std::endl;
        }
    }
    ImGuiFileDialog::Instance()->Close();
}

// ----------------------------------------------------------------------------
void HMI::loadRobot() const
{
    IGFD::FileDialogConfig cfg;
    ImGuiFileDialog::Instance()->OpenDialog(
        "RobotURDFDlg", "Select URDF", "URDF files{.urdf}", cfg);
}

// ----------------------------------------------------------------------------
void HMI::exportRobot() const
{
    if (m_selected_robot.empty())
        return;

    IGFD::FileDialogConfig cfg;
    cfg.sidePaneWidth = 0.0f;
    cfg.flags =
        ImGuiFileDialogFlags_ConfirmOverwrite | ImGuiFileDialogFlags_Modal;
    ImGuiFileDialog::Instance()->OpenDialog(
        "RobotExportDlg", "Export Robot", "DOT files{.dot}", cfg);
}

// ----------------------------------------------------------------------------
void HMI::exportRobotPanel()
{
    if (!ImGuiFileDialog::Instance()->Display("RobotExportDlg"))
        return;

    if (ImGuiFileDialog::Instance()->IsOk())
    {
        std::string filename = ImGuiFileDialog::Instance()->GetFilePathName();

        // Get the current robot
        auto const* controlled_robot =
            m_controller.getControlledRobot(m_selected_robot);
        if (controlled_robot == nullptr)
        {
            std::cerr << "❌ No robot selected for export" << std::endl;
            ImGuiFileDialog::Instance()->Close();
            return;
        }

        // Create exporter based on file extension
        auto exporter = RobotExporterFactory::create(filename);
        if (exporter == nullptr)
        {
            std::cerr << "❌ Unsupported export format: " << filename
                      << std::endl;
            ImGuiFileDialog::Instance()->Close();
            return;
        }

        // Export the robot
        if (exporter->exportTo(*controlled_robot, filename))
        {
            std::cout << "✅ Exported robot to: " << filename << std::endl;
        }
        else
        {
            std::cerr << "❌ Failed to export robot: " << exporter->error()
                      << std::endl;
        }
    }
    ImGuiFileDialog::Instance()->Close();
}

// ----------------------------------------------------------------------------
void HMI::removeRobot(std::string const& p_name)
{
    if (!m_robot_manager.removeRobot(p_name))
        return;

    if (m_selected_robot == p_name)
    {
        m_selected_robot.clear();
    }
    refreshRobotList();
    refreshCurrentRobotCaches();
}

// ----------------------------------------------------------------------------
void HMI::endEffectorSelectionPanel()
{
    auto const* controlled_robot =
        m_controller.getControlledRobot(m_selected_robot);
    if (controlled_robot == nullptr)
        return;

    std::vector<std::string> const& nodes = m_node_names;
    std::vector<std::string> const& end_effectors = m_end_effector_names;

    std::string current_end_effector =
        (controlled_robot->end_effector != nullptr)
            ? controlled_robot->end_effector->name()
            : "";

    ImGui::Text("End Effector Selection");
    if (ImGui::BeginCombo("End Effector",
                          current_end_effector.empty()
                              ? "Select..."
                              : current_end_effector.c_str()))
    {
        if (!end_effectors.empty())
        {
            renderEndEffectorNode("-- End Effectors --",
                                  ImVec4(0.5f, 1.0f, 0.5f, 1.0f),
                                  current_end_effector,
                                  true);
            drawEndEffectorCombo(end_effectors, current_end_effector);
        }

        renderEndEffectorNode("-- All Nodes --",
                              ImVec4(0.7f, 0.7f, 0.7f, 1.0f),
                              current_end_effector,
                              true);
        drawAllNodesCombo(nodes, end_effectors, current_end_effector);
        ImGui::EndCombo();
    }
}

// ----------------------------------------------------------------------------
void HMI::renderEndEffectorNode(std::string const& p_node_name,
                                ImVec4 p_color,
                                std::string const& p_current_end_effector,
                                bool p_is_header)
{
    if (p_is_header)
    {
        ImGui::PushStyleColor(ImGuiCol_Text, p_color);
        ImGui::Selectable(
            p_node_name.c_str(), false, ImGuiSelectableFlags_Disabled);
        ImGui::PopStyleColor();
        ImGui::Separator();
        return;
    }

    ImGui::PushID(p_node_name.c_str());
    bool is_selected = (p_current_end_effector == p_node_name);
    if (ImGui::Selectable(p_node_name.c_str(), is_selected))
    {
        if (m_controller.setEndEffector(m_selected_robot, p_node_name))
        {
            std::cout << "🎯 End effector set to: " << p_node_name << std::endl;
        }
    }
    if (is_selected)
        ImGui::SetItemDefaultFocus();
    ImGui::PopID();
}

// ----------------------------------------------------------------------------
void HMI::drawEndEffectorCombo(std::vector<std::string> const& p_end_effectors,
                               std::string const& p_current_end_effector)
{
    for (const auto& node_name : p_end_effectors)
    {
        renderEndEffectorNode(
            node_name, ImVec4(0.5f, 1.0f, 0.5f, 1.0f), p_current_end_effector);
    }
}

// ----------------------------------------------------------------------------
void HMI::drawAllNodesCombo(std::vector<std::string> const& p_nodes,
                            std::vector<std::string> const& p_end_effectors,
                            std::string const& p_current_end_effector)
{
    for (const auto& node_name : p_nodes)
    {
        if (std::find(p_end_effectors.begin(),
                      p_end_effectors.end(),
                      node_name) == p_end_effectors.end())
        {
            renderEndEffectorNode(node_name,
                                  ImVec4(0.7f, 0.7f, 0.7f, 1.0f),
                                  p_current_end_effector);
        }
    }
}

// ----------------------------------------------------------------------------
void HMI::teachPendantPanel()
{
    auto* robot = m_controller.getControlledRobot(m_selected_robot);
    if (robot == nullptr)
        return;

    auto* teach_pendant = m_controller.getTeachPendant();
    if (teach_pendant == nullptr)
        return;

    auto* ik_solver = m_controller.getIKSolver();
    if (ik_solver == nullptr)
        return;

    // Configure the teach pendant for this robot
    teach_pendant->setRobot(*robot);
    teach_pendant->setIKSolver(ik_solver);
    if (robot->end_effector != nullptr)
    {
        teach_pendant->setEndEffector(*robot->end_effector);
    }

    // Draw control mode tabs (notebook)
    auto selected_mode = drawControlModeTabs(robot->control_mode);
    robot->control_mode = selected_mode;

    ImGui::Spacing();

    // Joint Control Section (show in JOINT mode)
    if (selected_mode == ControlledRobot::ControlMode::JOINT)
    {
        drawJointControlSection(robot, teach_pendant);
        ImGui::Spacing();
    }

    // Cartesian Control Section (show in CARTESIAN mode)
    if (selected_mode == ControlledRobot::ControlMode::CARTESIAN)
    {
        drawCartesianControlSection(robot, teach_pendant);
        ImGui::Spacing();
    }
}

// ----------------------------------------------------------------------------
ControlledRobot::ControlMode
HMI::drawControlModeTabs(ControlledRobot::ControlMode p_current_mode) const
{
    auto selected_mode = p_current_mode;

    if (ImGui::BeginTabBar("ControlModeTabs"))
    {
        if (ImGui::BeginTabItem("Joint"))
        {
            if (selected_mode != ControlledRobot::ControlMode::JOINT)
            {
                selected_mode = ControlledRobot::ControlMode::JOINT;
                std::cout << "🎮 Control mode: Joint" << std::endl;
            }
            ImGui::EndTabItem();
        }

        if (ImGui::BeginTabItem("Cartesian"))
        {
            if (selected_mode != ControlledRobot::ControlMode::CARTESIAN)
            {
                selected_mode = ControlledRobot::ControlMode::CARTESIAN;
                std::cout << "🎮 Control mode: Cartesian" << std::endl;
            }
            ImGui::EndTabItem();
        }

        ImGui::EndTabBar();
    }

    return selected_mode;
}

// ----------------------------------------------------------------------------
void HMI::drawJointControlSection(ControlledRobot* p_robot,
                                  robotik::TeachPendant* p_teach_pendant) const
{
    ImGui::Text("Joint Control - %zu joints:",
                p_robot->blueprint().numJoints());

    // Add buttons for Neutral and Home positions
    if (ImGui::Button("Neutral", ImVec2(100, 0)))
    {
        p_robot->setNeutralPosition();
        std::cout << "🏠 Set neutral position" << std::endl;
    }
    ImGui::SameLine();
    if (ImGui::Button("Home", ImVec2(100, 0)))
    {
        p_robot->setHomePosition();
        std::cout << "🏠 Applied home position" << std::endl;
    }

    ImGui::BeginChild("JointList", ImVec2(0, 300), true);

    p_robot->blueprint().forEachJoint(
        [p_teach_pendant](robotik::Joint const& joint, size_t index)
        {
            ImGui::PushID(joint.name().c_str());

            auto value = static_cast<float>(joint.position());
            auto const [min, max] = joint.limits();

            if (ImGui::SliderFloat(joint.name().c_str(),
                                   &value,
                                   static_cast<float>(min),
                                   static_cast<float>(max),
                                   "%.3f"))
            {
                double delta = static_cast<double>(value) - joint.position();
                p_teach_pendant->moveJoint(index, delta, 1.0);
            }

            ImGui::PopID();
        });

    ImGui::EndChild();
}

// ----------------------------------------------------------------------------
void HMI::drawCartesianControlSection(
    ControlledRobot* p_robot,
    robotik::TeachPendant* p_teach_pendant) const
{
    ImGui::Text("Cartesian Control - Teach Pendant Style");

    drawFrameSelection(p_robot);

    ImGui::Spacing();

    // Use columns to put translation and rotation side by side
    ImGui::Columns(2, "CartesianControls", false);
    ImGui::SetColumnWidth(0, ImGui::GetWindowWidth() * 0.5f - 10.0f);

    // Left column: Translation
    drawTranslationControls(p_teach_pendant);

    ImGui::NextColumn();

    // Right column: Rotation
    drawRotationControls(p_teach_pendant);

    ImGui::Columns(1);
}

// ----------------------------------------------------------------------------
void HMI::drawFrameSelection(ControlledRobot* p_robot) const
{
    if (p_robot == nullptr)
        return;

    std::string current_frame = (p_robot->cartesian_frame != nullptr)
                                    ? p_robot->cartesian_frame->name()
                                    : "";

    if (ImGui::BeginCombo(
            "Frame", current_frame.empty() ? "World" : current_frame.c_str()))
    {
        // Option "World" (nullptr frame)
        bool is_world_selected = current_frame.empty();
        if (ImGui::Selectable("World", is_world_selected))
        {
            if (m_controller.setCartesianFrame(m_selected_robot, ""))
            {
                std::cout << "🎯 Cartesian frame set to: World" << std::endl;
            }
        }
        if (is_world_selected)
        {
            ImGui::SetItemDefaultFocus();
        }

        // All robot nodes
        for (const auto& node_name : m_node_names)
        {
            ImGui::PushID(node_name.c_str());
            bool is_selected = (current_frame == node_name);
            if (ImGui::Selectable(node_name.c_str(), is_selected))
            {
                if (m_controller.setCartesianFrame(m_selected_robot, node_name))
                {
                    std::cout << "🎯 Cartesian frame set to: " << node_name
                              << std::endl;
                }
            }
            if (is_selected)
            {
                ImGui::SetItemDefaultFocus();
            }
            ImGui::PopID();
        }
        ImGui::EndCombo();
    }
}

// ----------------------------------------------------------------------------
void HMI::drawTranslationControls(robotik::TeachPendant* p_teach_pendant) const
{
    ImGui::Text("Translation:");

    static int trans_axis = 0; // 0=X, 1=Y, 2=Z
    ImGui::RadioButton("X", &trans_axis, 0);
    ImGui::SameLine();
    ImGui::RadioButton("Y", &trans_axis, 1);
    ImGui::SameLine();
    ImGui::RadioButton("Z", &trans_axis, 2);

    // + / - buttons for translation
    static float step_size = 0.01f;
    ImGui::SetNextItemWidth(100);
    ImGui::InputFloat("Step (m)", &step_size, 0.001f, 0.01f, "%.3f");

    if (ImGui::Button("+##trans", ImVec2(50, 50)))
    {
        Eigen::Vector3d dir = Eigen::Vector3d::Zero();
        dir[trans_axis] = double(step_size);
        p_teach_pendant->moveCartesian(dir, 1.0);
    }
    ImGui::SameLine();
    if (ImGui::Button("-##trans", ImVec2(50, 50)))
    {
        Eigen::Vector3d dir = Eigen::Vector3d::Zero();
        dir[trans_axis] = -double(step_size);
        p_teach_pendant->moveCartesian(dir, 1.0);
    }
}

// ----------------------------------------------------------------------------
void HMI::drawRotationControls(robotik::TeachPendant* p_teach_pendant) const
{
    ImGui::Text("Rotation:");

    static int rot_axis = 0; // 0=X, 1=Y, 2=Z
    ImGui::RadioButton("X##rot", &rot_axis, 0);
    ImGui::SameLine();
    ImGui::RadioButton("Y##rot", &rot_axis, 1);
    ImGui::SameLine();
    ImGui::RadioButton("Z##rot", &rot_axis, 2);

    // + / - buttons for rotation
    static float angle_step = 0.05f;
    ImGui::SetNextItemWidth(100);
    ImGui::InputFloat("Step (rad)", &angle_step, 0.01f, 0.1f, "%.3f");

    if (ImGui::Button("+##rot", ImVec2(50, 50)))
    {
        Eigen::Vector3d axis = Eigen::Vector3d::Zero();
        axis[rot_axis] = 1.0;
        p_teach_pendant->rotateCartesian(axis, double(angle_step), 1.0);
    }
    ImGui::SameLine();
    if (ImGui::Button("-##rot", ImVec2(50, 50)))
    {
        Eigen::Vector3d axis = Eigen::Vector3d::Zero();
        axis[rot_axis] = 1.0;
        p_teach_pendant->rotateCartesian(axis, -double(angle_step), 1.0);
    }
}

// ----------------------------------------------------------------------------
void HMI::drawWaypointsSection(ControlledRobot* p_robot,
                               robotik::TeachPendant* p_teach_pendant) const
{
    ImGui::Text("Waypoints");
    ImGui::Separator();

    static std::string waypoint_label;
    static float waypoint_duration = 3.0f;
    std::vector<size_t> indices_to_delete;

    // Waypoint label and duration
    ImGui::SetNextItemWidth(150);
    ImGui::InputText("Label", &waypoint_label);
    ImGui::SameLine();
    ImGui::SetNextItemWidth(100);
    ImGui::InputFloat("Duration (s)", &waypoint_duration, 0.1f, 1.0f, "%.2f");
    ImGui::SameLine();
    if (ImGui::Button("Add"))
    {
        size_t idx = p_teach_pendant->recordWaypoint(
            waypoint_label, static_cast<double>(waypoint_duration));
        std::cout << "📍 Recorded waypoint " << idx << ": " << waypoint_label
                  << std::endl;
    }

    ImGui::Separator();
    ImGui::Text("Saved Waypoints (%zu):", p_robot->waypoints.size());
    ImGui::BeginChild("WaypointList", ImVec2(0, 200), true);

    // Display waypoints and collect indices to delete
    for (size_t i = 0; i < p_robot->waypoints.size(); ++i)
    {
        ImGui::PushID(static_cast<int>(i));

        // Put buttons first
        if (ImGui::Button("Go##waypoint", ImVec2(50, 0)))
        {
            if (p_teach_pendant->goToWaypoint(i,
                                              p_robot->waypoints[i].duration))
            {
                std::cout << "🎯 Going to waypoint " << i << std::endl;
            }
            else
            {
                std::cout << "⚠️ Failed to go to waypoint " << i << std::endl;
            }
        }
        ImGui::SameLine();
        if (ImGui::Button("Delete##waypoint", ImVec2(60, 0)))
        {
            // Collect index to delete (don't delete while iterating)
            indices_to_delete.push_back(i);
            std::cout << "🗑️ Marked waypoint " << i << " for deletion"
                      << std::endl;
        }
        ImGui::SameLine();

        // Waypoint name editing
        ImGui::SetNextItemWidth(150);
        ImGui::InputText("##name", &p_robot->waypoints[i].label);
        ImGui::SameLine();

        // Waypoint duration editing
        ImGui::InputDouble(
            "##duration", &p_robot->waypoints[i].duration, 0.1, 1.0, "%.2f");

        ImGui::PopID();
    }

    // Delete waypoints after the loop, in descending order to avoid index
    // shifting issues
    if (!indices_to_delete.empty())
    {
        // Sort in descending order
        std::sort(indices_to_delete.begin(),
                  indices_to_delete.end(),
                  std::greater<size_t>());

        // Remove duplicates (in case of multiple clicks)
        indices_to_delete.erase(
            std::unique(indices_to_delete.begin(), indices_to_delete.end()),
            indices_to_delete.end());

        // Delete from largest index to smallest
        for (size_t idx : indices_to_delete)
        {
            if (idx < p_robot->waypoints.size())
            {
                p_teach_pendant->deleteWaypoint(idx);
                std::cout << "🗑️ Deleted waypoint " << idx << std::endl;
            }
        }
        indices_to_delete.clear();
    }

    ImGui::EndChild();

    // Waypoint actions
    if (ImGui::Button("Clear All"))
    {
        p_teach_pendant->clearWaypoints();
        std::cout << "🗑️ Cleared all waypoints" << std::endl;
    }
}

// ----------------------------------------------------------------------------
void HMI::drawTrajectoryPlaybackSection(
    ControlledRobot* p_robot,
    robotik::TeachPendant* p_teach_pendant) const
{
    ImGui::Text("Trajectory Playback");
    ImGui::Separator();

    // Speed Factor
    ImGui::SetNextItemWidth(100);
    auto speed = static_cast<float>(p_robot->speed_factor);
    if (ImGui::SliderFloat("Speed", &speed, 0.0f, 1.0f, "%.2f"))
    {
        p_robot->speed_factor = static_cast<double>(speed);
    }

    // Loop the trajectory
    ImGui::SameLine();
    ImGui::Checkbox("Loop Trajectory", &p_robot->play_in_loop);

    // Play/Stop button
    bool is_playing = (p_robot->state == ControlledRobot::State::PLAYING);
    if (!is_playing)
    {
        if (ImGui::Button("▶ Play Trajectory", ImVec2(-1, 40)))
        {
            if (p_teach_pendant->playRecordedTrajectory())
            {
                std::cout << "▶️ Playing trajectory" << std::endl;
            }
            else
            {
                std::cout
                    << "⚠️ No trajectory to play (need at least 2 waypoints)"
                    << std::endl;
            }
        }
    }
    else
    {
        if (ImGui::Button("⏹ Stop", ImVec2(-1, 40)))
        {
            p_teach_pendant->stopTrajectory();
            std::cout << "⏹️ Stopped trajectory" << std::endl;
        }
    }

    // State and waypoint information below the button
    ImGui::Text("State: %s", is_playing ? "Playing" : "Idle");

    // Display waypoint information when playing
    if (is_playing && !p_robot->waypoints.empty())
    {
        bool has_error = false;
        bool target_reached = false;

        if (!p_robot->trajectory)
        {
            has_error = true;
        }
        else
        {
            // Check if target waypoint has been reached using pose error
            size_t target_wp_idx = p_robot->target_waypoint_index;
            if (target_wp_idx < p_robot->waypoints.size())
            {
                target_reached = robotik::TeachPendant::isWaypointReached(
                    p_robot, target_wp_idx);
            }
        }

        // Get target waypoint (destination)
        size_t target_wp_idx = p_robot->target_waypoint_index;
        std::string target_wp_name = "Unknown";
        if (target_wp_idx < p_robot->waypoints.size())
        {
            target_wp_name = p_robot->waypoints[target_wp_idx].label;
        }

        // Display waypoint information
        if (has_error)
        {
            ImGui::Text("Target Waypoint: ");
            ImGui::SameLine();
            ImGui::TextColored(
                ImVec4(1.0f, 0.0f, 0.0f, 1.0f), "%s", target_wp_name.c_str());
            ImGui::SameLine();
            ImGui::TextColored(ImVec4(1.0f, 0.0f, 0.0f, 1.0f), " (Error)");
        }
        else
        {
            // Show current waypoint if we're coming from a recorded waypoint
            // (current_waypoint_index != -1 means we're at a waypoint)
            if (p_robot->current_waypoint_index != -1)
            {
                size_t current_wp_idx =
                    static_cast<size_t>(p_robot->current_waypoint_index);
                if (current_wp_idx < p_robot->waypoints.size())
                {
                    std::string current_wp_name =
                        p_robot->waypoints[current_wp_idx].label;
                    ImGui::Text("Current Waypoint: ");
                    ImGui::SameLine();
                    ImGui::Text("%s", current_wp_name.c_str());
                }
            }

            // Display target waypoint with status
            ImGui::Text("Target Waypoint: ");
            ImGui::SameLine();
            if (target_reached)
            {
                ImGui::TextColored(ImVec4(0.0f, 1.0f, 0.0f, 1.0f),
                                   "%s",
                                   target_wp_name.c_str());
                ImGui::SameLine();
                ImGui::TextColored(ImVec4(0.0f, 1.0f, 0.0f, 1.0f),
                                   " (Reached)");
            }
            else
            {
                ImGui::Text("%s", target_wp_name.c_str());
            }
        }
    }
}

// ----------------------------------------------------------------------------
void HMI::cameraTargetPanel()
{
    auto* controlled_robot = m_controller.getControlledRobot(m_selected_robot);
    if (controlled_robot == nullptr)
        return;

    std::string current_camera_target =
        controlled_robot->camera_target
            ? controlled_robot->camera_target->name()
            : "";

    ImGui::Text("Camera Target Selection");
    drawCameraTargetCombo(current_camera_target);
    ImGui::Spacing();
    drawCameraTrackingCheckbox(controlled_robot);
}

// ----------------------------------------------------------------------------
void HMI::drawCameraTargetCombo(
    std::string const& p_current_camera_target) const
{
    if (ImGui::BeginCombo("Camera Target",
                          p_current_camera_target.empty()
                              ? "None"
                              : p_current_camera_target.c_str()))
    {
        for (const auto& node_name : m_node_names)
        {
            ImGui::PushID(node_name.c_str());
            bool is_selected = (p_current_camera_target == node_name);
            if (ImGui::Selectable(node_name.c_str(), is_selected))
            {
                if (m_controller.setCameraTarget(m_selected_robot, node_name))
                {
                    // Update camera controller target via Application
                    auto const* updated_robot =
                        m_controller.getControlledRobot(m_selected_robot);
                    if (updated_robot && updated_robot->camera_target)
                    {
                        Eigen::Vector3d target_pos =
                            updated_robot->camera_target->worldTransform()
                                .block<3, 1>(0, 3);
                        // The target will be updated in Application::onUpdate()
                        // via camera tracking, but we can also set it directly
                        // here For now, let the tracking handle it
                    }
                    std::cout << "📹 Camera target set to: " << node_name
                              << std::endl;
                }
            }
            if (is_selected)
            {
                ImGui::SetItemDefaultFocus();
            }
            ImGui::PopID();
        }
        ImGui::EndCombo();
    }
}

// ----------------------------------------------------------------------------
void HMI::drawCameraTrackingCheckbox(ControlledRobot* p_robot) const
{
    if (ImGui::Checkbox("Enable Camera Tracking",
                        &p_robot->camera_tracking_enabled))
    {
        std::cout << "📹 Camera tracking: "
                  << (p_robot->camera_tracking_enabled ? "enabled" : "disabled")
                  << std::endl;
    }
}

// ----------------------------------------------------------------------------
void HMI::drawCameraControllerPanel() const
{
    // Create a grid of buttons (3 columns) with Orbit + predefined views
    if (ImGui::BeginTable("CameraViews", 3, ImGuiTableFlags_None))
    {
        // Row 1: Orbit, Top, Bottom
        ImGui::TableNextColumn();
        if (ImGui::Button("Orbit", ImVec2(-FLT_MIN, 0)))
        {
            m_application.switchToOrbitController();
            std::cout << "📹 Switched to Orbit Controller" << std::endl;
        }

        ImGui::TableNextColumn();
        if (ImGui::Button("Top", ImVec2(-FLT_MIN, 0)))
        {
            m_application.setTopView();
            std::cout << "📹 Set to Top view" << std::endl;
        }

        ImGui::TableNextColumn();
        if (ImGui::Button("Bottom", ImVec2(-FLT_MIN, 0)))
        {
            m_application.setBottomView();
            std::cout << "📹 Set to Bottom view" << std::endl;
        }

        // Row 2: Front, Back, Right
        ImGui::TableNextColumn();
        if (ImGui::Button("Front", ImVec2(-FLT_MIN, 0)))
        {
            m_application.setFrontView();
            std::cout << "📹 Set to Front view" << std::endl;
        }

        ImGui::TableNextColumn();
        if (ImGui::Button("Back", ImVec2(-FLT_MIN, 0)))
        {
            m_application.setBackView();
            std::cout << "📹 Set to Back view" << std::endl;
        }

        ImGui::TableNextColumn();
        if (ImGui::Button("Right", ImVec2(-FLT_MIN, 0)))
        {
            m_application.setRightView();
            std::cout << "📹 Set to Right view" << std::endl;
        }

        // Row 3: Left (and potentially more buttons in the future)
        ImGui::TableNextColumn();
        if (ImGui::Button("Left", ImVec2(-FLT_MIN, 0)))
        {
            m_application.setLeftView();
            std::cout << "📹 Set to Left view" << std::endl;
        }

        ImGui::EndTable();
    }
}

// ----------------------------------------------------------------------------
void HMI::drawPredefinedViews() const
{
    // This method is now merged into drawCameraControllerPanel
    // Kept for compatibility, just call the main method
    drawCameraControllerPanel();
}

// ----------------------------------------------------------------------------
std::vector<std::string> HMI::getNodeNames() const
{
    return m_node_names;
}

// ----------------------------------------------------------------------------
std::vector<std::string> HMI::getEndEffectorNames() const
{
    return m_end_effector_names;
}

// ----------------------------------------------------------------------------
void HMI::refreshRobotList()
{
    m_robot_list.clear();
    m_robot_list.reserve(m_robot_manager.robots().size());
    for (const auto& [name, _] : m_robot_manager.robots())
    {
        m_robot_list.emplace_back(name);
    }
}

// ----------------------------------------------------------------------------
void HMI::refreshCurrentRobotCaches()
{
    m_node_names.clear();
    m_end_effector_names.clear();

    if (m_selected_robot.empty())
    {
        return;
    }

    auto* controlled_robot = m_controller.getControlledRobot(m_selected_robot);
    if (controlled_robot == nullptr || !controlled_robot->blueprint().hasRoot())
    {
        return;
    }

    m_node_names.reserve(controlled_robot->blueprint().numLinks());
    controlled_robot->blueprint().root().traverse(
        [this](robotik::Node const& node, size_t /*depth*/)
        { m_node_names.emplace_back(node.name()); });

    m_end_effector_names.reserve(
        controlled_robot->blueprint().endEffectors().size());
    for (auto const& end_eff : controlled_robot->blueprint().endEffectors())
    {
        m_end_effector_names.emplace_back(end_eff.get().name());
    }
}

// ----------------------------------------------------------------------------
bool HMI::setSelectedRobot(std::string const& name)
{
    if (m_selected_robot == name)
        return false;

    m_selected_robot = name;
    refreshCurrentRobotCaches();
    return true;
}

//------------------------------------------------------------------------------
void HMI::about() const
{
    ImVec2 center = ImGui::GetMainViewport()->GetCenter();
    ImGui::SetNextWindowPos(center, ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
    if (ImGui::BeginPopupModal(
            "About Robotik", nullptr, ImGuiWindowFlags_AlwaysAutoResize))
    {
        ImGui::Text("A robot control and visualization application");
        ImGui::Separator();
        std::string version(
            "Version: " + std::to_string(project::info::version::major) + '.' +
            std::to_string(project::info::version::minor) + '.' +
            std::to_string(project::info::version::patch));
        ImGui::Text("%s", version.c_str());
        ImGui::Separator();
        ImGui::Text("https://github.com/Lecrapouille/Robotik");
        ImGui::Text("Git branch: %s", project::info::git::branch.c_str());
        ImGui::Text("Git SHA1: %s", project::info::git::sha1.c_str());
        ImGui::Text("Compiled as %s",
                    (project::info::compilation::mode ==
                     project::info::compilation::Mode::debug)
                        ? "Debug"
                        : "Release");
        ImGui::Separator();
        ImGui::Text("Developed by Quentin Quadrat");
        ImGui::Text("Email: lecrapouille@gmail.com");
        ImGui::Separator();

        if (ImGui::Button("OK", ImVec2(120, 0)))
        {
            ImGui::CloseCurrentPopup();
        }

        ImGui::EndPopup();
    }
}

} // namespace robotik::application
