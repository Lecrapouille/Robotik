/**
 * @file HMI.cpp
 * @brief ImGui-based HMI for robot control and visualization implementation.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#include "HMI.hpp"

#include "ImGuiFileDialog/ImGuiFileDialog.h"
#include "Robotik/Core/Robot/Blueprint/Node.hpp"
#include "project_info.hpp"

#include <imgui.h>
#include <iostream>

namespace robotik::application
{

// ----------------------------------------------------------------------------
HMI::HMI(robotik::renderer::RobotManager& p_robot_manager,
         Controller& p_robot_controller,
         robotik::renderer::OrbitController& p_orbit_controller,
         std::function<void()> const& p_halt_callback)
    : m_robot_manager(p_robot_manager),
      m_controller(p_robot_controller),
      m_orbit_controller(p_orbit_controller),
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
    if (m_selected_robot.empty())
        return;

    ImGui::Begin("Camera Target");
    cameraTargetPanel();
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
}

// ----------------------------------------------------------------------------
void HMI::robotListPanel()
{
    ImGui::Text("Loaded Robots (%zu):", m_robot_list.size());
    ImGui::BeginChild("RobotList", ImVec2(0, 100), true);
    for (const auto& robot_name : m_robot_list)
    {
        if (ImGui::Selectable(robot_name.c_str(),
                              m_selected_robot == robot_name))
        {
            setSelectedRobot(robot_name);
        }
        if (ImGui::BeginPopupContextItem())
        {
            if (ImGui::MenuItem("Load robot"))
            {
                loadRobot();
            }
            if (ImGui::MenuItem("Remove robot"))
            {
                removeRobot(robot_name);
            }

            if (auto* controlled = m_controller.getControlledRobot(robot_name))
            {
                if (ImGui::MenuItem("Toggle Visibility"))
                {
                    controlled->blueprint().enable(
                        !controlled->blueprint().enabled());
                }
            }
            ImGui::EndPopup();
        }
    }
    ImGui::EndChild();
}

// ----------------------------------------------------------------------------
void HMI::loadRobotPanel()
{
    if (ImGuiFileDialog::Instance()->Display("RobotURDFDlg"))
    {
        if (ImGuiFileDialog::Instance()->IsOk())
        {
            std::string urdf = ImGuiFileDialog::Instance()->GetFilePathName();
            auto* robot = m_robot_manager.loadRobot(urdf);
            if (robot != nullptr)
            {
                std::cout << "✅ Loaded robot from: " << urdf << std::endl;

                // Extract blueprint and initialize controlled robot
                std::string robot_name = robot->name();
                robotik::Blueprint blueprint = std::move(robot->blueprint());
                m_controller.initializeRobot(
                    robot_name, std::move(blueprint), "", {}, "");

                setSelectedRobot(robot_name);
                refreshRobotList();
            }
            else
            {
                std::cerr << "❌ Failed to load robot: "
                          << m_robot_manager.error() << std::endl;
            }
        }
        ImGuiFileDialog::Instance()->Close();
    }
}

// ----------------------------------------------------------------------------
void HMI::loadRobot() const
{
    IGFD::FileDialogConfig cfg;
    ImGuiFileDialog::Instance()->OpenDialog(
        "RobotURDFDlg", "Select URDF", "URDF files{.urdf}", cfg);
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
    auto* controlled_robot = m_controller.getControlledRobot(m_selected_robot);
    if (controlled_robot == nullptr)
        return;

    std::vector<std::string> const& nodes = m_node_names;
    std::vector<std::string> const& end_effectors = m_end_effector_names;

    // TODO: Get current end effector name from teach pendant
    // Note: We don't have direct access to the end effector name from teach
    // pendant - we'll need to track it separately or add a getter
    std::string current_end_effector = "";

    ImGui::Text("End Effector Selection");
    if (ImGui::BeginCombo("End Effector",
                          current_end_effector.empty()
                              ? "Select..."
                              : current_end_effector.c_str()))
    {
        auto renderNode = [&](std::string const& node_name,
                              ImVec4 color,
                              bool is_header = false)
        {
            if (is_header)
            {
                ImGui::PushStyleColor(ImGuiCol_Text, color);
                ImGui::Selectable(
                    node_name.c_str(), false, ImGuiSelectableFlags_Disabled);
                ImGui::PopStyleColor();
                ImGui::Separator();
                return;
            }

            ImGui::PushID(node_name.c_str());
            bool is_selected = (current_end_effector == node_name);
            if (ImGui::Selectable(node_name.c_str(), is_selected))
            {
                if (m_controller.setEndEffector(m_selected_robot, node_name))
                {
                    std::cout << "🎯 End effector set to: " << node_name
                              << std::endl;
                }
            }
            if (is_selected)
                ImGui::SetItemDefaultFocus();
            ImGui::PopID();
        };

        if (!end_effectors.empty())
        {
            renderNode(
                "-- End Effectors --", ImVec4(0.5f, 1.0f, 0.5f, 1.0f), true);
            for (const auto& node_name : end_effectors)
                renderNode(node_name, ImVec4(0.5f, 1.0f, 0.5f, 1.0f));
        }

        renderNode("-- All Nodes --", ImVec4(0.7f, 0.7f, 0.7f, 1.0f), true);
        for (const auto& node_name : nodes)
        {
            if (std::find(end_effectors.begin(),
                          end_effectors.end(),
                          node_name) == end_effectors.end())
                renderNode(node_name, ImVec4(0.7f, 0.7f, 0.7f, 1.0f));
        }
        ImGui::EndCombo();
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

    // Configurer le teach pendant pour ce robot
    teach_pendant->setRobot(*robot);
    if (robot->end_effector != nullptr)
    {
        teach_pendant->setEndEffector(*robot->end_effector);
    }

    auto current_mode = robot->control_mode;

    // Control Mode Selection
    ImGui::Text("Control Mode:");
    ImGui::SameLine();
    int mode_idx = static_cast<int>(current_mode);
    if (ImGui::RadioButton("Joint", mode_idx == 0))
    {
        robot->control_mode =
            robotik::application::ControlledRobot::ControlMode::JOINT;
        std::cout << "🎮 Control mode: Joint" << std::endl;
    }
    ImGui::SameLine();
    if (ImGui::RadioButton("Cartesian", mode_idx == 1))
    {
        robot->control_mode =
            robotik::application::ControlledRobot::ControlMode::CARTESIAN;
        std::cout << "🎮 Control mode: Cartesian" << std::endl;
    }
    ImGui::SameLine();
    if (ImGui::RadioButton("Trajectory", mode_idx == 2))
    {
        robot->control_mode =
            robotik::application::ControlledRobot::ControlMode::TRAJECTORY;
        std::cout << "🎮 Control mode: Trajectory" << std::endl;
    }

    // Speed Factor (always visible)
    ImGui::SameLine();
    ImGui::SetNextItemWidth(100);
    float speed = static_cast<float>(robot->speed_factor);
    if (ImGui::SliderFloat("Speed", &speed, 0.0f, 1.0f, "%.2f"))
    {
        robot->speed_factor = static_cast<double>(speed);
    }

    ImGui::Separator();

    // Joint Control Section (show in JOINT mode)
    if (current_mode ==
        robotik::application::ControlledRobot::ControlMode::JOINT)
    {
        ImGui::Text("Joint Control - Joints (%zu):",
                    robot->blueprint().numJoints());
        ImGui::BeginChild("JointList", ImVec2(0, 300), true);

        robot->blueprint().forEachJoint(
            [teach_pendant](robotik::Joint& joint, size_t index)
            {
                ImGui::PushID(joint.name().c_str());

                auto value = static_cast<float>(joint.position());
                auto const [min, max] = joint.limits();

                // Format label with limits
                char label[256];
                snprintf(label,
                         sizeof(label),
                         "%s [%.3f, %.3f]",
                         joint.name().c_str(),
                         min,
                         max);

                if (ImGui::SliderFloat(label,
                                       &value,
                                       static_cast<float>(min),
                                       static_cast<float>(max),
                                       "%.3f"))
                {
                    double delta =
                        static_cast<double>(value) - joint.position();
                    teach_pendant->moveJoint(index, delta, 1.0);
                }

                ImGui::PopID();
            });

        ImGui::EndChild();
    }

    // Cartesian Control Section (show in CARTESIAN mode)
    if (current_mode ==
        robotik::application::ControlledRobot::ControlMode::CARTESIAN)
    {
        ImGui::Text("Cartesian Control - Teach Pendant Style");

        // Repère (référentiel)
        static int frame_idx = 0;
        const char* frames[] = { "World", "Tool", "Base" };
        ImGui::Combo("Repère", &frame_idx, frames, 3);

        ImGui::Separator();

        // Translation
        ImGui::Text("Translation:");
        static int trans_axis = 0; // 0=X, 1=Y, 2=Z
        ImGui::RadioButton("X", &trans_axis, 0);
        ImGui::SameLine();
        ImGui::RadioButton("Y", &trans_axis, 1);
        ImGui::SameLine();
        ImGui::RadioButton("Z", &trans_axis, 2);

        // Boutons + / - pour translation
        static float step_size = 0.01f;
        ImGui::SetNextItemWidth(100);
        ImGui::InputFloat("Step (m)", &step_size, 0.001f, 0.01f, "%.3f");

        if (ImGui::Button("+##trans", ImVec2(50, 50)))
        {
            Eigen::Vector3d dir = Eigen::Vector3d::Zero();
            dir[trans_axis] = step_size;
            teach_pendant->moveCartesian(dir, 1.0);
        }
        ImGui::SameLine();
        if (ImGui::Button("-##trans", ImVec2(50, 50)))
        {
            Eigen::Vector3d dir = Eigen::Vector3d::Zero();
            dir[trans_axis] = -step_size;
            teach_pendant->moveCartesian(dir, 1.0);
        }

        ImGui::Separator();

        // Rotation
        ImGui::Text("Rotation:");
        static int rot_axis = 0; // 0=X, 1=Y, 2=Z
        ImGui::RadioButton("X##rot", &rot_axis, 0);
        ImGui::SameLine();
        ImGui::RadioButton("Y##rot", &rot_axis, 1);
        ImGui::SameLine();
        ImGui::RadioButton("Z##rot", &rot_axis, 2);

        // Boutons + / - pour rotation
        static float angle_step = 0.05f;
        ImGui::SetNextItemWidth(100);
        ImGui::InputFloat("Step (rad)", &angle_step, 0.01f, 0.1f, "%.3f");

        if (ImGui::Button("+##rot", ImVec2(50, 50)))
        {
            Eigen::Vector3d axis = Eigen::Vector3d::Zero();
            axis[rot_axis] = 1.0;
            teach_pendant->rotateCartesian(axis, angle_step, 1.0);
        }
        ImGui::SameLine();
        if (ImGui::Button("-##rot", ImVec2(50, 50)))
        {
            Eigen::Vector3d axis = Eigen::Vector3d::Zero();
            axis[rot_axis] = 1.0;
            teach_pendant->rotateCartesian(axis, -angle_step, 1.0);
        }
    }

    ImGui::Separator();

    // Waypoints Section (always visible)
    if (ImGui::CollapsingHeader("Waypoints", ImGuiTreeNodeFlags_DefaultOpen))
    {
        static char waypoint_label[128] = "";

        ImGui::InputText("Label", waypoint_label, IM_ARRAYSIZE(waypoint_label));
        if (ImGui::Button("Record Waypoint"))
        {
            size_t idx = teach_pendant->recordWaypoint(waypoint_label);
            std::cout << "📍 Recorded waypoint " << idx << ": "
                      << waypoint_label << std::endl;
            waypoint_label[0] = '\0'; // Clear input
        }

        ImGui::Separator();
        ImGui::Text("Saved Waypoints (%zu):", robot->waypoints.size());
        ImGui::BeginChild("WaypointList", ImVec2(0, 200), true);

        auto const& waypoints = robot->waypoints;
        static int selected_waypoint = -1;

        for (size_t i = 0; i < waypoints.size(); ++i)
        {
            ImGui::PushID(static_cast<int>(i));

            bool is_selected = (selected_waypoint == static_cast<int>(i));
            std::string label =
                "[" + std::to_string(i) + "] waypoint_" + std::to_string(i);
            if (ImGui::Selectable(label.c_str(), is_selected))
            {
                selected_waypoint = static_cast<int>(i);
            }

            ImGui::SameLine();
            if (ImGui::SmallButton("Go"))
            {
                if (teach_pendant->goToWaypoint(i, 3.0))
                {
                    std::cout << "🎯 Going to waypoint " << i << std::endl;
                }
                else
                {
                    std::cout << "⚠️ Failed to go to waypoint " << i
                              << std::endl;
                }
            }
            ImGui::SameLine();
            if (ImGui::SmallButton("Delete"))
            {
                teach_pendant->deleteWaypoint(i);
                std::cout << "🗑️ Deleted waypoint " << i << std::endl;
                if (selected_waypoint == static_cast<int>(i))
                    selected_waypoint = -1;
            }

            ImGui::PopID();
        }

        ImGui::EndChild();

        // Waypoint actions
        if (ImGui::Button("Clear All"))
        {
            teach_pendant->clearWaypoints();
            selected_waypoint = -1;
            std::cout << "🗑️ Cleared all waypoints" << std::endl;
        }
    }

    // Trajectory Playback Section (show in TRAJECTORY mode or always visible)
    ImGui::Separator();
    if (ImGui::CollapsingHeader("Trajectory Playback"))
    {
        bool is_playing =
            (robot->state ==
             robotik::application::ControlledRobot::State::PLAYING);

        ImGui::Text("State: %s", is_playing ? "Playing" : "Idle");

        static bool loop_trajectory = false;
        ImGui::Checkbox("Loop Trajectory", &loop_trajectory);

        if (!is_playing)
        {
            if (ImGui::Button("▶ Play Trajectory", ImVec2(-1, 40)))
            {
                if (teach_pendant->playRecordedTrajectory(loop_trajectory))
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
            ImGui::TextColored(ImVec4(0.0f, 1.0f, 0.0f, 1.0f), "▶ Playing...");
            if (ImGui::Button("⏹ Stop", ImVec2(-1, 40)))
            {
                teach_pendant->stopTrajectory();
                std::cout << "⏹️ Stopped trajectory" << std::endl;
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
    if (ImGui::BeginCombo("Camera Target",
                          current_camera_target.empty()
                              ? "None"
                              : current_camera_target.c_str()))
    {
        for (const auto& node_name : m_node_names)
        {
            ImGui::PushID(node_name.c_str());
            bool is_selected = (current_camera_target == node_name);
            if (ImGui::Selectable(node_name.c_str(), is_selected))
            {
                if (m_controller.setCameraTarget(m_selected_robot, node_name))
                {
                    // Update orbit controller target
                    auto* updated_robot =
                        m_controller.getControlledRobot(m_selected_robot);
                    if (updated_robot && updated_robot->camera_target)
                    {
                        Eigen::Vector3d target_pos =
                            updated_robot->camera_target->worldTransform()
                                .block<3, 1>(0, 3);
                        m_orbit_controller.setTarget(target_pos.cast<float>());
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

    // Add checkbox to enable/disable camera tracking
    if (ImGui::Checkbox("Enable Camera Tracking",
                        &controlled_robot->camera_tracking_enabled))
    {
        std::cout << "📹 Camera tracking: "
                  << (controlled_robot->camera_tracking_enabled ? "enabled"
                                                                : "disabled")
                  << std::endl;
    }
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
