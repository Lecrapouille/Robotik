/**
 * @file ImGuiView.cpp
 * @brief ImGui-based view for robot control and visualization implementation.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#include "ImGuiView.hpp"
#include "MainApplication.hpp"

#include "ImGuiFileDialog/ImGuiFileDialog.h"
#include "Robotik/Core/Exporters/RobotExporterFactory.hpp"
#include "Robotik/Core/Robot/Blueprint/Blueprint.hpp"
#include "Robotik/Core/Robot/TeachPendant.hpp"
#include "project_info.hpp"

#include <algorithm>
#include <imgui.h>
#include <imgui_stdlib.h>
#include <iostream>

namespace robotik::application
{

// ----------------------------------------------------------------------------
ImGuiView::ImGuiView(ApplicationController& p_controller,
                     robotik::renderer::RobotManager& p_robot_manager,
                     CameraViewModel& p_camera_model,
                     MainApplication& p_main_app,
                     std::function<void()> const& p_halt_callback)
    : m_robot_manager(p_robot_manager),
      m_controller(p_controller),
      m_camera_model(p_camera_model),
      m_main_app(p_main_app),
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
void ImGuiView::onDrawMenuBar()
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
void ImGuiView::onDrawMainPanel()
{
    ImGui::Begin("Teach Pendant");

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
void ImGuiView::onDrawRobotManagementWindow()
{
    ImGui::Begin("Robots");
    robotManagementPanel();
    ImGui::End();
}

// ----------------------------------------------------------------------------
void ImGuiView::onDrawCameraTargetWindow()
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
void ImGuiView::onDrawTrajectoryWindow()
{
    ImGui::Begin("Trajectory");

    if (!m_selected_robot.empty())
    {
        auto* robot = m_controller.getControlledRobot(m_selected_robot);
        if (robot != nullptr)
        {
            // Waypoints Section
            drawWaypointsSection(robot, &m_main_app);
            ImGui::Spacing();

            // Trajectory Playback Section
            drawTrajectoryPlaybackSection(robot, &m_main_app);
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
void ImGuiView::robotManagementPanel()
{
    if (ImGui::Button("Load Robot"))
    {
        loadRobot();
    }

    robotListPanel();
    loadRobotPanel();
    exportRobotPanel();

    ImGui::Separator();
    sceneGraphPanel();
}

// ----------------------------------------------------------------------------
void ImGuiView::robotListPanel()
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
void ImGuiView::drawRobotListItem(std::string const& p_robot_name)
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
void ImGuiView::drawRobotListContextMenu(std::string const& p_robot_name)
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
void ImGuiView::loadRobotPanel()
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
            m_controller.initializeRobot(*robot, "", {}, "");

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
void ImGuiView::loadRobot() const
{
    IGFD::FileDialogConfig cfg;
    ImGuiFileDialog::Instance()->OpenDialog(
        "RobotURDFDlg", "Select URDF", "URDF files{.urdf}", cfg);
}

// ----------------------------------------------------------------------------
void ImGuiView::exportRobot() const
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
void ImGuiView::exportRobotPanel()
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
void ImGuiView::removeRobot(std::string const& p_name)
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
void ImGuiView::endEffectorSelectionPanel()
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
void ImGuiView::renderEndEffectorNode(std::string const& p_node_name,
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
void ImGuiView::drawEndEffectorCombo(
    std::vector<std::string> const& p_end_effectors,
    std::string const& p_current_end_effector)
{
    for (const auto& node_name : p_end_effectors)
    {
        renderEndEffectorNode(
            node_name, ImVec4(0.5f, 1.0f, 0.5f, 1.0f), p_current_end_effector);
    }
}

// ----------------------------------------------------------------------------
void ImGuiView::drawAllNodesCombo(
    std::vector<std::string> const& p_nodes,
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
void ImGuiView::teachPendantPanel()
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
    teach_pendant->setIKSolver(ik_solver);

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
ControlledRobot::ControlMode ImGuiView::drawControlModeTabs(
    ControlledRobot::ControlMode p_current_mode) const
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
void ImGuiView::drawJointControlSection(
    ControlledRobot* p_robot,
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
        [this, p_teach_pendant, p_robot](robotik::Joint const& joint,
                                         size_t index)
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
                p_teach_pendant->moveJoint(
                    *p_robot, index, delta, p_robot->speed_factor);
            }

            ImGui::PopID();
        });

    ImGui::EndChild();
}

// ----------------------------------------------------------------------------
void ImGuiView::drawCartesianControlSection(
    ControlledRobot* p_robot,
    robotik::TeachPendant* p_teach_pendant)
{
    ImGui::Text("Cartesian Control");

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

    // Display error message in red if there is one
    if (!m_cartesian_error.empty())
    {
        ImGui::Spacing();
        ImGui::TextColored(ImVec4(1.0f, 0.0f, 0.0f, 1.0f),
                           "❌ Error: %s",
                           m_cartesian_error.c_str());
    }
}

// ----------------------------------------------------------------------------
void ImGuiView::drawFrameSelection(ControlledRobot* p_robot) const
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
void ImGuiView::drawTranslationControls(robotik::TeachPendant* p_teach_pendant)
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

    auto* robot = m_controller.getControlledRobot(m_selected_robot);
    if (!robot || !robot->end_effector)
        return;

    if (ImGui::Button("+##trans", ImVec2(50, 50)))
    {
        m_cartesian_error.clear();
        Eigen::Vector3d dir = Eigen::Vector3d::Zero();
        dir[trans_axis] = double(step_size);
        if (!p_teach_pendant->moveCartesian(*robot,
                                            robot->end_effector,
                                            dir,
                                            robot->speed_factor,
                                            robot->cartesian_frame))
        {
            m_cartesian_error = p_teach_pendant->error();
            std::cerr << "❌ Failed to move robot: " << m_cartesian_error
                      << std::endl;
        }
    }
    ImGui::SameLine();
    if (ImGui::Button("-##trans", ImVec2(50, 50)))
    {
        m_cartesian_error.clear();
        Eigen::Vector3d dir = Eigen::Vector3d::Zero();
        dir[trans_axis] = -double(step_size);
        if (!p_teach_pendant->moveCartesian(*robot,
                                            robot->end_effector,
                                            dir,
                                            robot->speed_factor,
                                            robot->cartesian_frame))
        {
            m_cartesian_error = p_teach_pendant->error();
            std::cerr << "❌ Failed to move robot: " << m_cartesian_error
                      << std::endl;
        }
    }
}

// ----------------------------------------------------------------------------
void ImGuiView::drawRotationControls(robotik::TeachPendant* p_teach_pendant)
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

    auto* robot = m_controller.getControlledRobot(m_selected_robot);
    if (!robot || !robot->end_effector)
        return;

    if (ImGui::Button("+##rot", ImVec2(50, 50)))
    {
        m_cartesian_error.clear();
        Eigen::Vector3d axis = Eigen::Vector3d::Zero();
        axis[rot_axis] = 1.0;
        if (!p_teach_pendant->rotateCartesian(*robot,
                                              robot->end_effector,
                                              axis,
                                              double(angle_step),
                                              robot->speed_factor,
                                              robot->cartesian_frame))
        {
            m_cartesian_error = p_teach_pendant->error();
            std::cerr << "❌ Failed to rotate robot: " << m_cartesian_error
                      << std::endl;
        }
    }
    ImGui::SameLine();
    if (ImGui::Button("-##rot", ImVec2(50, 50)))
    {
        m_cartesian_error.clear();
        Eigen::Vector3d axis = Eigen::Vector3d::Zero();
        axis[rot_axis] = 1.0;
        if (!p_teach_pendant->rotateCartesian(*robot,
                                              robot->end_effector,
                                              axis,
                                              -double(angle_step),
                                              robot->speed_factor,
                                              robot->cartesian_frame))
        {
            m_cartesian_error = p_teach_pendant->error();
            std::cerr << "❌ Failed to rotate robot: " << m_cartesian_error
                      << std::endl;
        }
    }
}

// ----------------------------------------------------------------------------
void ImGuiView::drawWaypointsSection(ControlledRobot* p_robot,
                                     MainApplication* p_main_app) const
{
    ImGui::Text("Waypoints");
    ImGui::Separator();

    if (!p_robot || !p_main_app || !p_robot->end_effector)
        return;

    auto* wm = p_main_app->getWaypointManager(p_robot);
    if (!wm)
        return;

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
        size_t idx = wm->addWaypoint(*p_robot,
                                     p_robot->end_effector,
                                     waypoint_label,
                                     static_cast<double>(waypoint_duration));
        p_main_app->updateWaypointRenderCache(p_robot, p_robot->end_effector);
        std::cout << "📍 Recorded waypoint " << idx << ": " << waypoint_label
                  << std::endl;
    }

    ImGui::Separator();
    auto const& waypoints = wm->getWaypoints(p_robot->end_effector);
    ImGui::Text("Saved Waypoints (%zu):", waypoints.size());
    ImGui::BeginChild("WaypointList", ImVec2(0, 200), true);

    // Display waypoints and collect indices to delete
    for (size_t i = 0; i < waypoints.size(); ++i)
    {
        ImGui::PushID(static_cast<int>(i));

        // Put buttons first
        if (ImGui::Button("Go##waypoint", ImVec2(50, 0)))
        {
            auto* tc = p_main_app->getTrajectoryController(p_robot);
            if (tc && tc->goToWaypoint(p_robot->states().joint_positions,
                                       waypoints[i].position,
                                       waypoints[i].duration))
            {
                p_robot->state = ControlledRobot::State::PLAYING;
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

        // Waypoint name - read only for now (can be made editable later)
        ImGui::SetNextItemWidth(150);
        ImGui::Text("%s", waypoints[i].label.c_str());
        ImGui::SameLine();

        // Waypoint duration - read only for now
        ImGui::Text("%.2f s", waypoints[i].duration);

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
            if (idx < waypoints.size())
            {
                wm->deleteWaypoint(p_robot->end_effector, idx);
                p_main_app->updateWaypointRenderCache(p_robot,
                                                      p_robot->end_effector);
                std::cout << "🗑️ Deleted waypoint " << idx << std::endl;
            }
        }
        indices_to_delete.clear();
    }

    ImGui::EndChild();

    // Waypoint actions
    if (ImGui::Button("Clear All"))
    {
        wm->clearWaypoints(p_robot->end_effector);
        p_main_app->updateWaypointRenderCache(p_robot, p_robot->end_effector);
        std::cout << "🗑️ Cleared all waypoints" << std::endl;
    }
}

// ----------------------------------------------------------------------------
void ImGuiView::drawTrajectoryPlaybackSection(ControlledRobot* p_robot,
                                              MainApplication* p_main_app) const
{
    ImGui::Text("Trajectory Playback");
    ImGui::Separator();

    if (!p_robot || !p_main_app || !p_robot->end_effector)
        return;

    auto* wm = p_main_app->getWaypointManager(p_robot);
    auto* tc = p_main_app->getTrajectoryController(p_robot);
    if (!wm || !tc)
        return;

    // Speed Factor
    ImGui::SetNextItemWidth(100);
    auto speed = static_cast<float>(p_robot->speed_factor);
    if (ImGui::SliderFloat("Speed", &speed, 0.0f, 1.0f, "%.2f"))
    {
        p_robot->speed_factor = static_cast<double>(speed);
    }

    // Get waypoints
    auto const& waypoints = wm->getWaypoints(p_robot->end_effector);

    // Play/Stop button
    bool is_playing = (p_robot->state == ControlledRobot::State::PLAYING);
    if (!is_playing)
    {
        if (ImGui::Button("▶ Play Trajectory", ImVec2(-1, 40)))
        {
            if (waypoints.size() > 0)
            {
                // Use loop flag from UI (to be added if needed)
                bool loop = false; // or get from a checkbox
                if (tc->playWaypoints(
                        p_robot->states().joint_positions, waypoints, loop))
                {
                    p_robot->state = ControlledRobot::State::PLAYING;
                    std::cout << "▶️ Playing trajectory" << std::endl;
                }
            }
            else
            {
                std::cout << "⚠️ No waypoints to play" << std::endl;
            }
        }
    }
    else
    {
        if (ImGui::Button("⏹ Stop", ImVec2(-1, 40)))
        {
            tc->stop();
            p_robot->state = ControlledRobot::State::IDLE;
            std::cout << "⏹️ Stopped trajectory" << std::endl;
        }
    }

    // State and waypoint information below the button
    ImGui::Text("State: %s", is_playing ? "Playing" : "Idle");

    // Display waypoint information when playing
    if (is_playing && !waypoints.empty())
    {
        int current_wp_idx = tc->currentWaypointIndex();
        int target_wp_idx = tc->targetWaypointIndex();

        // Show current waypoint if valid
        if (current_wp_idx >= 0 &&
            static_cast<size_t>(current_wp_idx) < waypoints.size())
        {
            ImGui::Text("Current Waypoint: %s",
                        waypoints[current_wp_idx].label.c_str());
        }

        // Display target waypoint
        if (target_wp_idx >= 0 &&
            static_cast<size_t>(target_wp_idx) < waypoints.size())
        {
            ImGui::Text("Target Waypoint: %s",
                        waypoints[target_wp_idx].label.c_str());
        }
    }
}

// ----------------------------------------------------------------------------
void ImGuiView::cameraTargetPanel()
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
void ImGuiView::drawCameraTargetCombo(
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
                    // The target will be updated in Application::onUpdate()
                    // via camera tracking
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
void ImGuiView::drawCameraTrackingCheckbox(ControlledRobot* p_robot) const
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
void ImGuiView::drawCameraControllerPanel() const
{
    // Create a grid of buttons (3 columns) with Orbit + predefined views
    if (ImGui::BeginTable("CameraViews", 3, ImGuiTableFlags_None))
    {
        // Row 1: Orbit, Top, Bottom
        ImGui::TableNextColumn();
        if (ImGui::Button("Orbit", ImVec2(-FLT_MIN, 0)))
        {
            m_camera_model.setView(CameraViewModel::ViewType::ORBIT);
            std::cout << "📹 Switched to Orbit Controller" << std::endl;
        }

        ImGui::TableNextColumn();
        if (ImGui::Button("Top", ImVec2(-FLT_MIN, 0)))
        {
            m_camera_model.setView(CameraViewModel::ViewType::TOP);
            std::cout << "📹 Set to Top view" << std::endl;
        }

        ImGui::TableNextColumn();
        if (ImGui::Button("Bottom", ImVec2(-FLT_MIN, 0)))
        {
            m_camera_model.setView(CameraViewModel::ViewType::BOTTOM);
            std::cout << "📹 Set to Bottom view" << std::endl;
        }

        // Row 2: Front, Back, Right
        ImGui::TableNextColumn();
        if (ImGui::Button("Front", ImVec2(-FLT_MIN, 0)))
        {
            m_camera_model.setView(CameraViewModel::ViewType::FRONT);
            std::cout << "📹 Set to Front view" << std::endl;
        }

        ImGui::TableNextColumn();
        if (ImGui::Button("Back", ImVec2(-FLT_MIN, 0)))
        {
            m_camera_model.setView(CameraViewModel::ViewType::BACK);
            std::cout << "📹 Set to Back view" << std::endl;
        }

        ImGui::TableNextColumn();
        if (ImGui::Button("Right", ImVec2(-FLT_MIN, 0)))
        {
            m_camera_model.setView(CameraViewModel::ViewType::RIGHT);
            std::cout << "📹 Set to Right view" << std::endl;
        }

        // Row 3: Left (and potentially more buttons in the future)
        ImGui::TableNextColumn();
        if (ImGui::Button("Left", ImVec2(-FLT_MIN, 0)))
        {
            m_camera_model.setView(CameraViewModel::ViewType::LEFT);
            std::cout << "📹 Set to Left view" << std::endl;
        }

        ImGui::EndTable();
    }
}

// ----------------------------------------------------------------------------
void ImGuiView::drawPredefinedViews() const
{
    // This method is now merged into drawCameraControllerPanel
    // Kept for compatibility, just call the main method
    drawCameraControllerPanel();
}

// ----------------------------------------------------------------------------
std::vector<std::string> ImGuiView::getNodeNames() const
{
    return m_node_names;
}

// ----------------------------------------------------------------------------
std::vector<std::string> ImGuiView::getEndEffectorNames() const
{
    return m_end_effector_names;
}

// ----------------------------------------------------------------------------
void ImGuiView::refreshRobotList()
{
    m_robot_list.clear();
    m_robot_list.reserve(m_robot_manager.robots().size());
    for (const auto& [name, _] : m_robot_manager.robots())
    {
        m_robot_list.emplace_back(name);
    }
}

// ----------------------------------------------------------------------------
void ImGuiView::refreshCurrentRobotCaches()
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
bool ImGuiView::setSelectedRobot(std::string const& name)
{
    if (m_selected_robot == name)
        return false;

    m_selected_robot = name;
    refreshCurrentRobotCaches();
    return true;
}

//------------------------------------------------------------------------------
void ImGuiView::about() const
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

// ----------------------------------------------------------------------------
void ImGuiView::sceneGraphPanel()
{
    if (m_selected_robot.empty())
    {
        ImGui::TextColored(ImVec4(1.0f, 1.0f, 0.0f, 1.0f),
                           "Select a robot to view scene graph");
        return;
    }

    auto* robot = m_controller.getControlledRobot(m_selected_robot);
    if (robot == nullptr)
    {
        return;
    }

    ImGui::Text("Scene Graph");
    ImGui::BeginChild("SceneGraph", ImVec2(0, 200), true);

    auto const& root = robot->blueprint().root();
    drawSceneGraphNode(root, nullptr);

    ImGui::EndChild();

    // Add Frame button
    if (ImGui::Button("Add Frame"))
    {
        m_show_add_frame_dialog = true;
    }

    // Add Frame dialog
    if (m_show_add_frame_dialog)
    {
        ImGui::OpenPopup("Add Frame");
        m_show_add_frame_dialog = false;
    }

    if (ImGui::BeginPopupModal(
            "Add Frame", nullptr, ImGuiWindowFlags_AlwaysAutoResize))
    {
        static char frame_name[256] = "";
        static float translation[3] = { 0.0f, 0.0f, 0.0f };
        static float rotation[3] = { 0.0f,
                                     0.0f,
                                     0.0f }; // Euler angles in degrees

        ImGui::InputText("Name", frame_name, sizeof(frame_name));
        ImGui::InputFloat3("Translation", translation);
        ImGui::InputFloat3("Rotation (deg)", rotation);

        if (ImGui::Button("Cancel"))
        {
            ImGui::CloseCurrentPopup();
            frame_name[0] = '\0';
            translation[0] = translation[1] = translation[2] = 0.0f;
            rotation[0] = rotation[1] = rotation[2] = 0.0f;
        }

        ImGui::SameLine();
        if (ImGui::Button("Create"))
        {
            if (frame_name[0] != '\0')
            {
                // Create transform from translation and rotation
                Eigen::Vector3d trans(
                    translation[0], translation[1], translation[2]);
                Eigen::Vector3d rot_rad(
                    static_cast<double>(rotation[0]) * M_PI / 180.0,
                    static_cast<double>(rotation[1]) * M_PI / 180.0,
                    static_cast<double>(rotation[2]) * M_PI / 180.0);

                // Create rotation matrix from Euler angles (ZYX order)
                Eigen::Matrix3d rot =
                    (Eigen::AngleAxisd(rot_rad.z(), Eigen::Vector3d::UnitZ()) *
                     Eigen::AngleAxisd(rot_rad.y(), Eigen::Vector3d::UnitY()) *
                     Eigen::AngleAxisd(rot_rad.x(), Eigen::Vector3d::UnitX()))
                        .toRotationMatrix();

                using robotik::Transform;
                Transform transform = Transform::Identity();
                transform.block<3, 3>(0, 0) = rot;
                transform.block<3, 1>(0, 3) = trans;

                robot->addFrame(std::string(frame_name), transform);

                ImGui::CloseCurrentPopup();
                frame_name[0] = '\0';
                translation[0] = translation[1] = translation[2] = 0.0f;
                rotation[0] = rotation[1] = rotation[2] = 0.0f;
            }
        }

        ImGui::EndPopup();
    }
}

// ----------------------------------------------------------------------------
void ImGuiView::drawSceneGraphNode(::robotik::Node const& p_node,
                                   ::robotik::Node const* /*p_parent*/)
{
    using namespace robotik;

    // Determine node type
    std::string node_type = "Node";
    ImVec4 type_color(0.7f, 0.7f, 0.7f, 1.0f);

    if (dynamic_cast<Frame const*>(&p_node))
    {
        node_type = "Frame";
        type_color = ImVec4(1.0f, 1.0f, 0.0f, 1.0f); // Yellow
    }
    else if (dynamic_cast<Joint const*>(&p_node))
    {
        node_type = "Joint";
        type_color = ImVec4(0.0f, 1.0f, 1.0f, 1.0f); // Cyan
    }
    else if (dynamic_cast<Link const*>(&p_node))
    {
        node_type = "Link";
        type_color = ImVec4(0.0f, 1.0f, 0.0f, 1.0f); // Green
    }
    else if (dynamic_cast<Geometry const*>(&p_node))
    {
        node_type = "Geometry";
        type_color = ImVec4(1.0f, 0.5f, 0.0f, 1.0f); // Orange
    }

    // Create tree node label
    std::string label = "[" + node_type + "] " + p_node.name();

    // Check if node has children
    bool has_children = !p_node.children().empty();

    ImGuiTreeNodeFlags flags = ImGuiTreeNodeFlags_OpenOnArrow;
    if (!has_children)
    {
        flags |= ImGuiTreeNodeFlags_Leaf;
    }

    // Draw the node
    ImGui::PushStyleColor(ImGuiCol_Text, type_color);
    bool node_open = ImGui::TreeNodeEx(label.c_str(), flags);
    ImGui::PopStyleColor();

    if (node_open)
    {
        // Recursively draw children
        for (auto const& child : p_node.children())
        {
            drawSceneGraphNode(*child, child.get());
        }

        ImGui::TreePop();
    }
}

} // namespace robotik::application
