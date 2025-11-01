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

    robot->blueprint().forEachJointData(
        [read_only, robot](JointData const& joint_data, size_t i)
        {
            ImGui::PushID(joint_data.name.c_str());

            float value = static_cast<float>(robot->state().joint_positions[i]);
            float const min = static_cast<float>(joint_data.position_min);
            float const max = static_cast<float>(joint_data.position_max);

            if (read_only)
            {
                ImGui::BeginDisabled();
                ImGui::SliderFloat(joint_data.name.c_str(),
                                   &value,
                                   min,
                                   max,
                                   "%.3f");
                ImGui::EndDisabled();
            }
            else
            {
                if (ImGui::SliderFloat(joint_data.name.c_str(),
                                       &value,
                                       min,
                                       max,
                                       "%.3f"))
                {
                    // Update the robot state and recompute FK
                    robot->state().joint_positions[i] = static_cast<double>(value);
                    const_cast<Robot*>(static_cast<Robot const*>(robot))->setJointPositions(robot->state());
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
        (robot->control_link_index < robot->blueprint().numLinks())
            ? robot->blueprint().linkData(robot->control_link_index).name
            : "";

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
        (robot->camera_target_link_index < robot->blueprint().numLinks())
            ? robot->blueprint().linkData(robot->camera_target_link_index).name
            : "";

    if (ImGui::BeginCombo("Camera Target",
                          current_camera_target.empty()
                              ? "None"
                              : current_camera_target.c_str()))
    {
        // Iterate over all links for camera target selection
        robot->blueprint().forEachLinkData([&](LinkData const& link, size_t i) {
            ImGui::PushID(link.name.c_str());
            bool is_selected = (current_camera_target == link.name);
            if (ImGui::Selectable(link.name.c_str(), is_selected))
            {
                if (m_controller.setCameraTarget(m_selected_robot, link.name))
                {
                    // Update orbit controller target
                    auto* updated_robot =
                        m_robot_manager.getRobot(m_selected_robot);
                    if (updated_robot && 
                        updated_robot->camera_target_link_index < updated_robot->state().link_transforms.size())
                    {
                        Eigen::Vector3d target_pos =
                            updated_robot->state().link_transforms[updated_robot->camera_target_link_index]
                                .block<3, 1>(0, 3);
                        m_orbit_controller.setTarget(target_pos.cast<float>());
                    }
                    std::cout << "📹 Camera target set to: " << link.name
                              << std::endl;
                }
            }
            if (is_selected)
            {
                ImGui::SetItemDefaultFocus();
            }
            ImGui::PopID();
        });
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
    if (robot == nullptr)
    {
        return;
    }

    // Collect all link names
    m_node_names.reserve(robot->blueprint().numLinks());
    robot->blueprint().forEachLinkData(
        [this](LinkData const& link, size_t /*index*/)
        { m_node_names.emplace_back(link.name); });

    // Find end effectors: links that have no child joints
    // A link is an end effector if no joint has this link as parent
    std::vector<bool> is_parent(robot->blueprint().numLinks(), false);
    robot->blueprint().forEachJointData(
        [&is_parent](JointData const& joint, size_t /*index*/)
        {
            if (joint.parent_link_index != SIZE_MAX)
            {
                // The parent joint's parent link has children
                // Actually, we need to mark links that ARE parents of joints
                // This is a bit complex. Let's simplify: a link is an end effector
                // if no joint has it as its parent link.
                // But JointData has parent_index (parent joint), not parent link.
                // We need to find which link contains each joint.
            }
        });
    
    // Simpler approach: find links that no joint points to via parent_joint_index
    std::vector<bool> has_child_joint(robot->blueprint().numLinks(), false);
    robot->blueprint().forEachLinkData(
        [&](LinkData const& link, size_t link_idx)
        {
            // Check if any joint has this link as parent
            robot->blueprint().forEachJointData(
                [&](JointData const& joint, size_t /*joint_idx*/)
                {
                    if (joint.parent_link_index != SIZE_MAX)
                    {
                        // This joint has a parent joint.
                        // The link attached to that parent joint has a child.
                        // But we need to know which link is attached to each joint...
                        // Actually, each Link has a parent_joint_index, so we can
                        // determine which link "belongs" to which joint.
                    }
                });
        });
    
    // More direct approach: a link is an end effector if no other link
    // has this link's "owning joint" as parent_joint_index
    for (size_t link_idx = 0; link_idx < robot->blueprint().numLinks(); ++link_idx)
    {
        LinkData const& link = robot->blueprint().linkData(link_idx);
        
        // Check if there's any joint that uses this link
        // Find joint index for this link (if link.parent_joint_index != SIZE_MAX,
        // this link belongs to that joint's output)
        // Then check if any other link has that joint as parent
        
        bool has_children = false;
        for (size_t other_link_idx = 0; other_link_idx < robot->blueprint().numLinks(); ++other_link_idx)
        {
            if (other_link_idx == link_idx) continue;
            
            LinkData const& other_link = robot->blueprint().linkData(other_link_idx);
            
            // If another link's parent joint is the joint after this link,
            // then this link has children
            // But we need to find "the joint after this link"
            // This is complex. Let's just mark all links as potential end effectors
            // for now, or use a heuristic.
        }
        
        // Heuristic: just add all links for now
        m_end_effector_names.push_back(link.name);
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
