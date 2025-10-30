/**
 * @file DearRobotHMI.cpp
 * @brief ImGui-based HMI for robot control and visualization implementation.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#include "DearRobotHMI.hpp"
#include "ImGuiFileDialog/ImGuiFileDialog.h"
#include "Robotik/Core/Common/Conversions.hpp"
#include "Robotik/Core/Robot/Blueprint/Blueprint.hpp"
#include "Robotik/Core/Robot/Blueprint/Joint.hpp"
#include "Robotik/Core/Robot/Blueprint/Node.hpp"
#include "Robotik/Core/Solvers/IKSolver.hpp"
#include "project_info.hpp"

#include <imgui.h>
#include <iostream>

namespace robotik::application
{

// ----------------------------------------------------------------------------
DearRobotHMI::DearRobotHMI(
    robotik::renderer::RobotManager& p_robot_manager,
    robotik::renderer::OrbitController& p_orbit_controller,
    std::function<void()> const& p_halt_callback)
    : m_robot_manager(p_robot_manager),
      m_orbit_controller(p_orbit_controller),
      m_halt_callback(p_halt_callback)
{
    // Initialize selected robot to current robot if any
    if (auto const* robot = m_robot_manager.currentRobot(); robot != nullptr)
    {
        m_selected_robot = robot->name();
    }
    refreshRobotList();
    refreshCurrentRobotCaches();
}

// ----------------------------------------------------------------------------
void DearRobotHMI::onDrawMenuBar()
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
void DearRobotHMI::onDrawMainPanel()
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
void DearRobotHMI::robotManagementPanel()
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
void DearRobotHMI::robotListPanel()
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
void DearRobotHMI::loadRobotPanel()
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
                initializeRobotConfigurations(*robot);
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
void DearRobotHMI::loadRobot() const
{
    IGFD::FileDialogConfig cfg;
    ImGuiFileDialog::Instance()->OpenDialog(
        "RobotURDFDlg", "Select URDF", "URDF files{.urdf}", cfg);
}

// ----------------------------------------------------------------------------
void DearRobotHMI::removeRobot(std::string const& p_name)
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
void DearRobotHMI::controlModePanel()
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
        robot->control_mode =
            static_cast<renderer::RobotManager::ControlMode>(current_mode);
        std::cout << "🎮 Control mode changed to: "
                  << s_mode_names[current_mode] << std::endl;
    }
}

// ----------------------------------------------------------------------------
void DearRobotHMI::jointControlPanel()
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
void DearRobotHMI::endEffectorPanel()
{
    if (!ImGui::CollapsingHeader("End Effectors"))
        return;

    auto* robot = m_robot_manager.getRobot(m_selected_robot);
    if (robot == nullptr)
        return;

    std::vector<std::string> const& nodes = m_node_names;
    std::vector<std::string> const& end_effectors = m_end_effector_names;

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
            ImGui::PushStyleColor(ImGuiCol_Text,
                                  ImVec4(0.5f, 1.0f, 0.5f, 1.0f));
            ImGui::PushID("##EffectorHeader");
            ImGui::Selectable(
                "-- End Effectors --", false, ImGuiSelectableFlags_Disabled);
            ImGui::PopID();
            ImGui::PopStyleColor();

            for (const auto& node_name : end_effectors)
            {
                ImGui::PushID(node_name.c_str());
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
                ImGui::PopID();
            }
            ImGui::Separator();
        }

        // Then show all other nodes
        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.7f, 0.7f, 0.7f, 1.0f));
        ImGui::PushID("##AllNodesHeader");
        ImGui::Selectable(
            "-- All Nodes --", false, ImGuiSelectableFlags_Disabled);
        ImGui::PopID();
        ImGui::PopStyleColor();

        for (const auto& node_name : nodes)
        {
            // Skip if already shown in end effectors
            if (std::find(end_effectors.begin(),
                          end_effectors.end(),
                          node_name) != end_effectors.end())
            {
                continue;
            }

            ImGui::PushID(node_name.c_str());
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
            ImGui::PopID();
        }
        ImGui::EndCombo();
    }
}

// ----------------------------------------------------------------------------
void DearRobotHMI::cameraTargetPanel()
{
    if (!ImGui::CollapsingHeader("Camera Targets"))
        return;

    auto* robot = m_robot_manager.getRobot(m_selected_robot);
    if (robot == nullptr)
        return;

    std::vector<std::string> const& nodes = m_node_names;

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
            ImGui::PushID(node_name.c_str());
            bool is_selected = (current_camera_target == node_name);
            if (ImGui::Selectable(node_name.c_str(), is_selected))
            {
                robot->camera_target =
                    Node::find(robot->blueprint().root(), node_name);
                if (robot->camera_target)
                {
                    // Enable camera tracking when a target is selected
                    robot->camera_tracking_enabled = true;
                    // Update orbit controller target
                    Eigen::Vector3d target_pos =
                        robot->camera_target->worldTransform().block<3, 1>(0,
                                                                           3);
                    m_orbit_controller.setTarget(target_pos.cast<float>());
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
std::vector<std::string> DearRobotHMI::getNodeNames() const
{
    return m_node_names;
}

// ----------------------------------------------------------------------------
std::vector<std::string> DearRobotHMI::getEndEffectorNames() const
{
    return m_end_effector_names;
}

// ----------------------------------------------------------------------------
void DearRobotHMI::refreshRobotList()
{
    m_robot_list.clear();
    m_robot_list.reserve(m_robot_manager.robots().size());
    for (const auto& [name, _] : m_robot_manager.robots())
    {
        m_robot_list.emplace_back(name);
    }
}

// ----------------------------------------------------------------------------
void DearRobotHMI::refreshCurrentRobotCaches()
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
bool DearRobotHMI::setSelectedRobot(std::string const& name)
{
    if (m_selected_robot == name)
        return false;

    m_selected_robot = name;
    refreshCurrentRobotCaches();
    return true;
}

// ----------------------------------------------------------------------------
void DearRobotHMI::initializeRobotConfigurations(
    renderer::RobotManager::ControlledRobot& p_robot)
{
    // Set control joint (end effector) for inverse kinematics
    // Use the first end effector if available, otherwise use root
    p_robot.control_joint = nullptr;
    if (auto const& end_effectors = p_robot.blueprint().endEffectors();
        !end_effectors.empty())
    {
        p_robot.control_joint = &end_effectors[0].get();
        std::cout << "🤖 Control joint set to: "
                  << p_robot.control_joint->name() << std::endl;
    }
    else
    {
        p_robot.control_joint = &p_robot.blueprint().root();
        std::cout << "🤖 Control joint set to root: "
                  << p_robot.control_joint->name() << std::endl;
    }

    // Compute IK target poses if control joint is set
    if (p_robot.control_joint != nullptr)
    {
        std::cout << "🎯 Computing IK target poses..." << std::endl;

        // Reserve memory for 3 joint configurations
        size_t const num_poses = 3;
        std::vector<std::vector<double>> joint_configs(num_poses);
        for (auto& it : joint_configs)
        {
            it.resize(p_robot.blueprint().numJoints());
        }

        // Setup 3 joint configurations: 1/3 joint limits, 2/3 joint limits and
        // neutral position
        p_robot.blueprint().forEachJoint(
            [&joint_configs](Joint const& joint, size_t index)
            {
                auto [min, max] = joint.limits();

                // Pose 1: 1/3 from min
                joint_configs[0][index] = min + (max - min) * 1.0 / 3.0;
                // Pose 2: center (neutral)
                joint_configs[1][index] = (max + min) / 2.0;
                // Pose 3: 2/3 from min
                joint_configs[2][index] = min + (max - min) * 2.0 / 3.0;
            });

        // Save current joint positions
        std::vector<double> saved_positions;
        saved_positions.reserve(p_robot.blueprint().numJoints());
        p_robot.blueprint().forEachJoint(
            [&saved_positions](Joint const& joint, size_t /*index*/)
            { saved_positions.push_back(joint.position()); });

        // For each configuration, compute end-effector pose
        p_robot.ik_target_poses.clear();
        p_robot.ik_target_poses.reserve(num_poses);
        for (size_t pose_id = 0; pose_id < num_poses; ++pose_id)
        {
            // Apply joint configuration
            p_robot.blueprint().forEachJoint(
                [&joint_configs, pose_id](Joint& joint, size_t index)
                { joint.position(joint_configs[pose_id][index]); });

            // Get end-effector transform
            Transform end_effector_transform =
                p_robot.control_joint->worldTransform();

            // Convert to pose and store
            Pose target_pose = robotik::transformToPose(end_effector_transform);
            p_robot.ik_target_poses.push_back(target_pose);

            std::cout << "  Target " << (pose_id + 1) << ": ["
                      << target_pose.transpose() << "]" << std::endl;
        }

        // Restore original joint positions
        p_robot.blueprint().forEachJoint(
            [&saved_positions](Joint& joint, size_t index)
            { joint.position(saved_positions[index]); });

        // Initialize IK solver
        p_robot.ik_solver = std::make_unique<JacobianIKSolver>();
    }

    // Compute trajectory configurations
    std::cout << "🎯 Computing trajectory configurations..." << std::endl;

    // Generate 3 joint configurations
    size_t const num_configs = 3;
    p_robot.trajectory_configs.clear();
    p_robot.trajectory_configs.resize(num_configs);

    for (auto& config : p_robot.trajectory_configs)
    {
        config.resize(p_robot.blueprint().numJoints());
    }

    // Setup 3 joint configurations: 1/4, 1/2, 3/4 of joint range
    p_robot.blueprint().forEachJoint(
        [&p_robot](Joint const& joint, size_t index)
        {
            auto [min, max] = joint.limits();

            // Config 0: 1/4 from min
            p_robot.trajectory_configs[0][index] = min + (max - min) * 0.25;
            // Config 1: center
            p_robot.trajectory_configs[1][index] = (max + min) / 2.0;
            // Config 2: 3/4 from min
            p_robot.trajectory_configs[2][index] = min + (max - min) * 0.75;
        });

    // Print configurations
    for (size_t i = 0; i < num_configs; ++i)
    {
        std::cout << "  Config " << (i + 1) << ": [";
        for (size_t j = 0; j < p_robot.trajectory_configs[i].size(); ++j)
        {
            std::cout << p_robot.trajectory_configs[i][j];
            if (j < p_robot.trajectory_configs[i].size() - 1)
                std::cout << ", ";
        }
        std::cout << "]" << std::endl;
    }
}

//------------------------------------------------------------------------------
void DearRobotHMI::about()
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
