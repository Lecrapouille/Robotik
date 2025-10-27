/**
 * @file DearRobotHMI.cpp
 * @brief ImGui-based HMI for robot control and visualization implementation.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#include "DearRobotHMI.hpp"
#include "Robotik/Robot/Blueprint/Blueprint.hpp"
#include "Robotik/Robot/Blueprint/Joint.hpp"

#include <imgui.h>
#include <iostream>

namespace robotik::renderer
{

// ----------------------------------------------------------------------------
DearRobotHMI::DearRobotHMI(RobotManager& p_robot_manager,
                           OrbitController& p_orbit_controller,
                           std::function<void()> p_halt_callback)
    : m_robot_manager(p_robot_manager),
      m_orbit_controller(p_orbit_controller),
      m_halt_callback(p_halt_callback)
{
    // Initialize selected robot to current robot if any
    if (auto* robot = m_robot_manager.currentRobot(); robot != nullptr)
    {
        m_selected_robot = robot->name();
    }
}

// ----------------------------------------------------------------------------
void DearRobotHMI::onDrawMainPanel()
{
    ImGui::Begin("Robot Control");

    robotManagementPanel();

    // Only show controls if a robot is selected
    if (!m_selected_robot.empty())
    {
        controlModePanel();
        endEffectorPanel();
        cameraTargetPanel();
        jointControlPanel();
    }

    ImGui::End();
}

// ----------------------------------------------------------------------------
void DearRobotHMI::onDrawMenuBar()
{
    if (ImGui::BeginMenu("File"))
    {
        if (ImGui::MenuItem("Quit"))
        {
            if (m_halt_callback)
            {
                m_halt_callback();
            }
        }
        ImGui::EndMenu();
    }
}

// ----------------------------------------------------------------------------
void DearRobotHMI::robotManagementPanel()
{
    if (ImGui::CollapsingHeader("Robot Management",
                                ImGuiTreeNodeFlags_DefaultOpen))
    {
        // Get list of robots
        std::vector<std::string> robot_list;
        for (const auto& [name, _] : m_robot_manager.robots())
        {
            robot_list.push_back(name);
        }

        // Robot selection
        if (ImGui::BeginCombo(
                "Selected Robot",
                m_selected_robot.empty() ? "None" : m_selected_robot.c_str()))
        {
            for (const auto& robot_name : robot_list)
            {
                bool is_selected = (m_selected_robot == robot_name);
                if (ImGui::Selectable(robot_name.c_str(), is_selected))
                {
                    m_selected_robot = robot_name;
                }
                if (is_selected)
                {
                    ImGui::SetItemDefaultFocus();
                }
            }
            ImGui::EndCombo();
        }

        ImGui::Separator();

        // Add Robot
        ImGui::Text("Add Robot:");
        ImGui::InputText(
            "URDF Path", m_urdf_path_buffer, sizeof(m_urdf_path_buffer));
        ImGui::SameLine();
        if (ImGui::Button("Browse..."))
        {
            // TODO: Add file browser dialog
            ImGui::OpenPopup("File Browser");
        }

        if (ImGui::Button("Load Robot"))
        {
            if (m_urdf_path_buffer[0] != '\0')
            {
                auto* robot =
                    m_robot_manager.loadRobot(std::string(m_urdf_path_buffer));
                if (robot != nullptr)
                {
                    std::cout << "✅ Loaded robot from: " << m_urdf_path_buffer
                              << std::endl;
                    m_selected_robot = robot->name();
                    // Clear buffer after successful load
                    m_urdf_path_buffer[0] = '\0';
                }
                else
                {
                    std::cerr << "❌ Failed to load robot: "
                              << m_robot_manager.error() << std::endl;
                }
            }
        }

        ImGui::SameLine();

        // Remove Robot
        if (ImGui::Button("Remove Robot"))
        {
            if (!m_selected_robot.empty())
            {
                if (m_robot_manager.removeRobot(m_selected_robot))
                {
                    std::cout << "✅ Removed robot: " << m_selected_robot
                              << std::endl;
                    m_selected_robot.clear();
                }
            }
        }

        // Display robot list
        ImGui::Text("Loaded Robots (%zu):", robot_list.size());
        ImGui::BeginChild("RobotList", ImVec2(0, 100), true);
        for (const auto& robot_name : robot_list)
        {
            if (ImGui::Selectable(robot_name.c_str(),
                                  m_selected_robot == robot_name))
            {
                m_selected_robot = robot_name;
            }
        }
        ImGui::EndChild();
    }
}

// ----------------------------------------------------------------------------
void DearRobotHMI::controlModePanel()
{
    if (ImGui::CollapsingHeader("Control Mode", ImGuiTreeNodeFlags_DefaultOpen))
    {
        auto* robot = m_robot_manager.getRobot(m_selected_robot);
        if (robot == nullptr)
            return;

        int current_mode = static_cast<int>(robot->control_mode);

        const char* mode_names[] = { "No Control",
                                     "Direct Kinematics",
                                     "Animation",
                                     "Inverse Kinematics",
                                     "Trajectory" };

        if (ImGui::Combo("Mode", &current_mode, mode_names, 5))
        {
            robot->control_mode =
                static_cast<RobotManager::ControlMode>(current_mode);
            std::cout << "🎮 Control mode changed to: "
                      << mode_names[current_mode] << std::endl;
        }
    }
}

// ----------------------------------------------------------------------------
void DearRobotHMI::jointControlPanel()
{
    if (ImGui::CollapsingHeader("Joint Control",
                                ImGuiTreeNodeFlags_DefaultOpen))
    {
        auto* robot = m_robot_manager.getRobot(m_selected_robot);
        if (robot == nullptr)
            return;

        std::vector<std::pair<std::string, double>> joints = getJoints();

        // Determine if sliders should be read-only
        int control_mode = static_cast<int>(robot->control_mode);
        // Sliders are editable only in DIRECT_KINEMATICS mode (1)
        bool read_only = (control_mode != 1);

        ImGui::Text("Joints (%zu):", joints.size());
        ImGui::BeginChild("JointList", ImVec2(0, 300), true);

        size_t joint_index = 0;
        robot->blueprint().forEachJoint(
            [&](Joint& joint, size_t /*index*/)
            {
                ImGui::PushID(joint.name().c_str());

                float value = static_cast<float>(joint.position());
                auto [min, max] = joint.limits();

                if (read_only)
                {
                    // Read-only mode: disable interaction
                    ImGui::BeginDisabled();
                    ImGui::SliderFloat(joint.name().c_str(),
                                       &value,
                                       static_cast<float>(min),
                                       static_cast<float>(max),
                                       "%.3f");
                    ImGui::EndDisabled();
                }
                else
                {
                    // Interactive mode
                    if (ImGui::SliderFloat(joint.name().c_str(),
                                           &value,
                                           static_cast<float>(min),
                                           static_cast<float>(max),
                                           "%.3f"))
                    {
                        joint.position(static_cast<double>(value));
                    }
                }

                ImGui::PopID();
                joint_index++;
            });

        ImGui::EndChild();
    }
}

// ----------------------------------------------------------------------------
void DearRobotHMI::endEffectorPanel()
{
    if (ImGui::CollapsingHeader("End Effector"))
    {
        auto* robot = m_robot_manager.getRobot(m_selected_robot);
        if (robot == nullptr)
            return;

        std::vector<std::string> nodes = getNodeNames();
        std::vector<std::string> end_effectors = getEndEffectorNames();

        std::string current_end_effector;
        if (robot->control_joint != nullptr)
        {
            current_end_effector = robot->control_joint->name();
        }

        if (ImGui::BeginCombo("End Effector",
                              current_end_effector.empty()
                                  ? "None"
                                  : current_end_effector.c_str()))
        {
            // First, show end effectors (highlighted)
            if (!end_effectors.empty())
            {
                ImGui::TextColored(ImVec4(0.5f, 1.0f, 0.5f, 1.0f),
                                   "-- End Effectors --");
                for (const auto& node_name : end_effectors)
                {
                    bool is_selected = (current_end_effector == node_name);
                    if (ImGui::Selectable(node_name.c_str(), is_selected))
                    {
                        robot->control_joint =
                            Node::find(robot->blueprint().root(), node_name);
                        if (robot->control_joint)
                        {
                            std::cout << "🎯 End effector set to: " << node_name
                                      << std::endl;
                        }
                    }
                    if (is_selected)
                    {
                        ImGui::SetItemDefaultFocus();
                    }
                }
                ImGui::Separator();
            }

            // Then show all other nodes
            ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.0f),
                               "-- All Nodes --");
            for (const auto& node_name : nodes)
            {
                // Skip if already shown in end effectors
                if (std::find(end_effectors.begin(),
                              end_effectors.end(),
                              node_name) != end_effectors.end())
                {
                    continue;
                }

                bool is_selected = (current_end_effector == node_name);
                if (ImGui::Selectable(node_name.c_str(), is_selected))
                {
                    robot->control_joint =
                        Node::find(robot->blueprint().root(), node_name);
                    if (robot->control_joint)
                    {
                        std::cout << "🎯 End effector set to: " << node_name
                                  << std::endl;
                    }
                }
                if (is_selected)
                {
                    ImGui::SetItemDefaultFocus();
                }
            }
            ImGui::EndCombo();
        }
    }
}

// ----------------------------------------------------------------------------
void DearRobotHMI::cameraTargetPanel()
{
    if (ImGui::CollapsingHeader("Camera Target"))
    {
        auto* robot = m_robot_manager.getRobot(m_selected_robot);
        if (robot == nullptr)
            return;

        std::vector<std::string> nodes = getNodeNames();

        std::string current_camera_target;
        if (robot->camera_target != nullptr)
        {
            current_camera_target = robot->camera_target->name();
        }

        if (ImGui::BeginCombo("Camera Target",
                              current_camera_target.empty()
                                  ? "None"
                                  : current_camera_target.c_str()))
        {
            for (const auto& node_name : nodes)
            {
                bool is_selected = (current_camera_target == node_name);
                if (ImGui::Selectable(node_name.c_str(), is_selected))
                {
                    robot->camera_target =
                        Node::find(robot->blueprint().root(), node_name);
                    if (robot->camera_target)
                    {
                        // Update orbit controller target
                        Eigen::Vector3d target_pos =
                            robot->camera_target->worldTransform().block<3, 1>(
                                0, 3);
                        m_orbit_controller.setTarget(target_pos.cast<float>());
                        std::cout << "📹 Camera target set to: " << node_name
                                  << std::endl;
                    }
                }
                if (is_selected)
                {
                    ImGui::SetItemDefaultFocus();
                }
            }
            ImGui::EndCombo();
        }
    }
}

// ----------------------------------------------------------------------------
std::vector<std::string> DearRobotHMI::getNodeNames() const
{
    std::vector<std::string> node_names;
    auto* robot = m_robot_manager.getRobot(m_selected_robot);
    if (robot != nullptr && robot->blueprint().hasRoot())
    {
        robot->blueprint().root().traverse(
            [&node_names](Node const& node, size_t /*depth*/)
            { node_names.push_back(node.name()); });
    }
    return node_names;
}

// ----------------------------------------------------------------------------
std::vector<std::string> DearRobotHMI::getEndEffectorNames() const
{
    std::vector<std::string> end_effector_names;
    auto* robot = m_robot_manager.getRobot(m_selected_robot);
    if (robot != nullptr && robot->blueprint().hasRoot())
    {
        for (auto const& end_eff : robot->blueprint().endEffectors())
        {
            end_effector_names.push_back(end_eff.get().name());
        }
    }
    return end_effector_names;
}

// ----------------------------------------------------------------------------
std::vector<std::pair<std::string, double>> DearRobotHMI::getJoints() const
{
    std::vector<std::pair<std::string, double>> joints;
    auto* robot = m_robot_manager.getRobot(m_selected_robot);
    if (robot != nullptr && robot->blueprint().hasRoot())
    {
        robot->blueprint().forEachJoint(
            [&joints](Joint const& joint, size_t /*index*/)
            { joints.emplace_back(joint.name(), joint.position()); });
    }
    return joints;
}

} // namespace robotik::renderer
