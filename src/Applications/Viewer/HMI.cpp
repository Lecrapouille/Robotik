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
    ImGui::Begin("Robot Control");

    robotManagementPanel();
    if (!m_selected_robot.empty())
    {
        cameraTargetPanel();
        controlModePanel();
        endEffectorPanel();
        jointControlPanel();
    }

    ImGui::End();

    // Render IK failure popup for selected robot (outside main window)
    if (!m_selected_robot.empty())
    {
        m_controller.renderIKFailurePopup(m_selected_robot);
    }
}

// ----------------------------------------------------------------------------
void HMI::robotManagementPanel()
{
    if (ImGui::CollapsingHeader("Robot Management",
                                ImGuiTreeNodeFlags_DefaultOpen))
    {
        if (ImGui::Button("Load Robot"))
        {
            loadRobot();
        }

        robotListPanel();
        loadRobotPanel();
    }
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

            if (auto const* robot = m_robot_manager.getRobot(robot_name);
                robot != nullptr)
            {
                if (ImGui::MenuItem("Hide robot", nullptr, !robot->is_visible))
                {
                    m_robot_manager.setRobotVisibility(robot_name, false);
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
                m_controller.initializeRobot(*robot);
                setSelectedRobot(robot->name());
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
void HMI::controlModePanel()
{
    static constexpr std::array<const char*, 5> s_mode_names = {
        "No Control",
        "Direct Kinematics",
        "Animation",
        "Inverse Kinematics",
        "Trajectory"
    };

    if (!ImGui::CollapsingHeader("Control Mode",
                                 ImGuiTreeNodeFlags_DefaultOpen))
        return;

    auto* robot = m_robot_manager.getRobot(m_selected_robot);
    if (robot == nullptr)
        return;

    auto current_mode = int(robot->control_mode);
    if (ImGui::Combo("Mode",
                     &current_mode,
                     s_mode_names.data(),
                     static_cast<int>(s_mode_names.size())))
    {
        auto new_mode =
            static_cast<renderer::RobotManager::ControlMode>(current_mode);
        m_controller.setControlMode(m_selected_robot, new_mode);
        std::cout << "🎮 Control mode changed to: "
                  << s_mode_names[current_mode] << std::endl;
    }
}

// ----------------------------------------------------------------------------
void HMI::jointControlPanel()
{
    if (!ImGui::CollapsingHeader("Joint Control",
                                 ImGuiTreeNodeFlags_DefaultOpen))
        return;

    auto* robot = m_robot_manager.getRobot(m_selected_robot);
    if (robot == nullptr)
        return;

    // Sliders are editable only in DIRECT_KINEMATICS mode
    bool read_only = (robot->control_mode !=
                      renderer::RobotManager::ControlMode::DIRECT_KINEMATICS);

    ImGui::Text("Joints (%zu):", robot->blueprint().numJoints());
    ImGui::BeginChild("JointList", ImVec2(0, 300), true);

    robot->blueprint().forEachJoint(
        [read_only](Joint& joint, size_t /*index*/)
        {
            ImGui::PushID(joint.name().c_str());

            auto value = static_cast<float>(joint.position());
            auto const [min, max] = joint.limits();

            if (read_only)
            {
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
        });

    ImGui::EndChild();
}

// ----------------------------------------------------------------------------
void HMI::endEffectorPanel()
{
    if (!ImGui::CollapsingHeader("End Effectors"))
        return;

    auto* robot = m_robot_manager.getRobot(m_selected_robot);
    if (robot == nullptr)
        return;

    std::vector<std::string> const& nodes = m_node_names;
    std::vector<std::string> const& end_effectors = m_end_effector_names;

    std::string current_end_effector =
        robot->control_link ? robot->control_link->name() : "";

    if (ImGui::BeginCombo("End Effector",
                          current_end_effector.empty()
                              ? "None"
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
                if (m_controller.setControlJoint(m_selected_robot, node_name))
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
void HMI::cameraTargetPanel()
{
    if (!ImGui::CollapsingHeader("Camera Targets"))
        return;

    auto* robot = m_robot_manager.getRobot(m_selected_robot);
    if (robot == nullptr)
        return;

    std::string current_camera_target =
        robot->camera_target ? robot->camera_target->name() : "";

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
                        m_robot_manager.getRobot(m_selected_robot);
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
                        &robot->camera_tracking_enabled))
    {
        std::cout << "📹 Camera tracking: "
                  << (robot->camera_tracking_enabled ? "enabled" : "disabled")
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

    auto* robot = m_robot_manager.getRobot(m_selected_robot);
    if (robot == nullptr || !robot->blueprint().hasRoot())
    {
        return;
    }

    m_node_names.reserve(robot->blueprint().numLinks());
    robot->blueprint().root().traverse(
        [this](Node const& node, size_t /*depth*/)
        { m_node_names.emplace_back(node.name()); });

    m_end_effector_names.reserve(robot->blueprint().endEffectors().size());
    for (auto const& end_eff : robot->blueprint().endEffectors())
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
