/**
 * @file Controller.cpp
 * @brief Controller for robot control logic implementation.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#include "Controller.hpp"
#include "Robotik/Core/Common/Conversions.hpp"
#include "Robotik/Core/Robot/Blueprint/Joint.hpp"
#include "Robotik/Core/Robot/Blueprint/Node.hpp"
#include "Robotik/Core/Solvers/IKSolver.hpp"
#include "Robotik/Core/Solvers/Trajectory.hpp"

#include <imgui.h>
#include <iostream>

namespace robotik::application
{

// ----------------------------------------------------------------------------
Controller::Controller(renderer::RobotManager& p_robot_manager)
    : m_robot_manager(p_robot_manager)
{
}

// ----------------------------------------------------------------------------
bool Controller::initializeRobots(std::string const& p_link_name)
{
    bool success = true;
    for (auto& [_, robot] : m_robot_manager.robots())
    {
        if (!initializeRobot(robot, p_link_name))
        {
            success = false;
        }
    }
    return success;
}

// ----------------------------------------------------------------------------
bool Controller::initializeRobot(
    renderer::RobotManager::ControlledRobot& p_robot,
    std::string const& p_link_name,
    JointValues const& p_initial_joint_positions)
{
    // Set initial joint values from configuration. If no configuration is
    // provided, set neutral position.
    if (!p_initial_joint_positions.empty())
    {
        p_robot.setJointPositions(p_initial_joint_positions);
        std::cout << "🤖 Set initial joint values from configuration"
                  << std::endl;
    }
    else
    {
        p_robot.setNeutralPosition();
        std::cout << "🤖 Set neutral position" << std::endl;
    }

    // Set control link (end effector) for inverse kinematics.
    // If no joint name is provided, use first end effector.
    // If joint name is provided, use it.
    // If joint name is not found, use base_link.
    p_robot.control_link = nullptr;
    if (!p_link_name.empty())
    {
        // Try to find the link by name.
        p_robot.control_link =
            Node::find(p_robot.blueprint().root(), p_link_name);

        // If link is not found, use base_link.
        if (p_robot.control_link == nullptr)
        {
            p_robot.control_link = &p_robot.blueprint().root();
        }
    }
    else
    {
        // Use first end effector if available, otherwise use root
        if (auto const& end_effectors = p_robot.blueprint().endEffectors();
            !end_effectors.empty())
        {
            p_robot.control_link = &end_effectors[0].get();
        }
        else
        {
            p_robot.control_link = &p_robot.blueprint().root();
        }
    }

    // No control link found, return false, the robot will not be loaded.
    if (p_robot.control_link != nullptr)
    {
        std::cout << "🤖 Control link set to: " << p_robot.control_link->name()
                  << std::endl;
    }
    else
    {
        std::cout << "🤖 Error: control link not found for robot: "
                  << p_robot.name() << std::endl;
        return false;
    }

    // Initialize robot state BEFORE computing IK targets (which needs to access
    // it)
    m_robot_states[p_robot.name()] = RobotState{};

    // Compute IK target poses if control link is set
    computeIKTargetPoses(p_robot);

    // Compute trajectory configurations
    computeTrajectoryConfigs(p_robot);

    return true;
}

// ----------------------------------------------------------------------------
void Controller::update(double p_elapsed_time, double p_dt)
{
    for (auto& [robot_name, robot] : m_robot_manager.robots())
    {
        auto& robot_state = m_robot_states[robot_name];

        switch (robot.control_mode)
        {
            case renderer::RobotManager::ControlMode::ANIMATION:
                handleAnimation(robot, p_elapsed_time);
                break;
            case renderer::RobotManager::ControlMode::INVERSE_KINEMATICS:
                handleInverseKinematics(robot, p_dt, robot_state);
                break;
            case renderer::RobotManager::ControlMode::TRAJECTORY:
                handleTrajectory(robot, p_elapsed_time, p_dt);
                break;
            case renderer::RobotManager::ControlMode::NO_CONTROL:
            case renderer::RobotManager::ControlMode::DIRECT_KINEMATICS:
            default:
                break;
        }
    }
}

// ----------------------------------------------------------------------------
bool Controller::setControlMode(std::string const& p_robot_name,
                                renderer::RobotManager::ControlMode p_mode)
{
    auto* robot = m_robot_manager.getRobot(p_robot_name);
    if (robot == nullptr)
        return false;

    robot->control_mode = p_mode;

    // Reset trajectory when switching to trajectory mode
    if (p_mode == renderer::RobotManager::ControlMode::TRAJECTORY)
    {
        robot->trajectory_segment = 0;
        robot->trajectory.reset();
    }

    // Reset IK cycle when switching to IK mode
    if (p_mode == renderer::RobotManager::ControlMode::INVERSE_KINEMATICS)
    {
        auto& robot_state = m_robot_states[p_robot_name];
        robot_state.ik_cycle_complete = false; // Allow restart
        robot_state.solution_computed = false; // Recompute solution
    }

    return true;
}

// ----------------------------------------------------------------------------
bool Controller::setControlJoint(std::string const& p_robot_name,
                                 std::string const& p_link_name)
{
    auto* robot = m_robot_manager.getRobot(p_robot_name);
    if (robot == nullptr)
        return false;

    robot->control_link = Node::find(robot->blueprint().root(), p_link_name);
    if (robot->control_link == nullptr)
        return false;

    return true;
}

// ----------------------------------------------------------------------------
bool Controller::setCameraTarget(std::string const& p_robot_name,
                                 std::string const& p_node_name)
{
    auto* robot = m_robot_manager.getRobot(p_robot_name);
    if (robot == nullptr)
        return false;

    robot->camera_target = Node::find(robot->blueprint().root(), p_node_name);
    if (robot->camera_target == nullptr)
        return false;

    robot->camera_tracking_enabled = true;
    return true;
}

// ----------------------------------------------------------------------------
void Controller::computeIKTargetPoses(
    renderer::RobotManager::ControlledRobot& p_robot)
{
    if (p_robot.control_link == nullptr)
        return;

    std::cout << "🎯 Computing IK target poses..." << std::endl;

    // Reserve memory for 3 joint configurations
    size_t const num_poses = 3;
    std::vector<std::vector<double>> joint_configs(num_poses);
    for (auto& it : joint_configs)
    {
        it.resize(p_robot.blueprint().numJoints());
    }

    // Setup 3 joint configurations: 1/3, center, 2/3 of joint limits
    p_robot.blueprint().forEachJoint(
        [&joint_configs](Joint const& joint, size_t index)
        {
            auto [min, max] = joint.limits();
            joint_configs[0][index] = min + (max - min) * 1.0 / 3.0;
            joint_configs[1][index] = (max + min) / 2.0;
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
            p_robot.control_link->worldTransform();

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
    p_robot.ik_solver = std::make_unique<robotik::JacobianIKSolver>();

    // Store home configuration (center position) in robot state
    auto& robot_state = m_robot_states[p_robot.name()];
    robot_state.home_configuration = joint_configs[1];
}

// ----------------------------------------------------------------------------
void Controller::computeTrajectoryConfigs(
    renderer::RobotManager::ControlledRobot& p_robot)
{
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
            p_robot.trajectory_configs[0][index] = min + (max - min) * 0.25;
            p_robot.trajectory_configs[1][index] = (max + min) / 2.0;
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

// ----------------------------------------------------------------------------
void Controller::handleAnimation(
    renderer::RobotManager::ControlledRobot& p_robot,
    double p_time) const
{
    // Update animation time (slower for more visible animation)
    float animation_time = static_cast<float>(p_time) * 0.5f;

    // Get current joint values
    auto& joint_values = p_robot.state().joint_positions;

    // Animate each joint with different frequencies and amplitudes
    for (size_t i = 0; i < joint_values.size(); ++i)
    {
        float frequency = 0.8f + static_cast<float>(i) * 0.3f;
        float amplitude = 0.8f;
        float phase = static_cast<float>(i) * 1.5f;
        joint_values[i] =
            double(amplitude * std::sin(frequency * animation_time + phase));
    }

    // Update joint positions
    p_robot.setJointPositions(p_robot.state());
}

// ----------------------------------------------------------------------------
void Controller::handleInverseKinematics(
    renderer::RobotManager::ControlledRobot& p_robot,
    [[maybe_unused]] double p_dt,
    RobotState& p_robot_state) const
{
    if (p_robot.control_link == nullptr || p_robot.ik_solver == nullptr)
    {
        std::cout << "🤖 Error: control link or solver not set" << std::endl;
        return;
    }

    // If cycle is complete, stop (user can manually restart)
    if (p_robot_state.ik_cycle_complete)
    {
        return; // Stay in IK mode but don't compute anything
    }

    // Get current target pose based on state
    Pose const& target_pose =
        p_robot.ik_target_poses[p_robot_state.target_pose_index];

    // Compute IK solution only once per target
    if (!p_robot_state.solution_computed)
    {
        bool solved = p_robot.ik_solver->solve(
            p_robot, *p_robot.control_link, target_pose);

        if (solved && p_robot.ik_solver->converged())
        {
            // IK converged successfully
            p_robot_state.ik_solution = p_robot.ik_solver->solution();
            p_robot_state.solution_computed = true;
            p_robot_state.ik_failed = false;

            std::cout << "🎯 IK solved for target "
                      << (p_robot_state.target_pose_index + 1)
                      << " - Error: " << p_robot.ik_solver->poseError()
                      << " - Iterations: " << p_robot.ik_solver->numIterations()
                      << std::endl;

            // Get current joint positions
            std::vector<double> current_positions;
            current_positions.reserve(p_robot.blueprint().numJoints());
            p_robot.blueprint().forEachJoint(
                [&current_positions](Joint const& joint, size_t /*index*/)
                { current_positions.push_back(joint.position()); });

            // Create trajectory to IK solution
            robotik::JointSpaceGenerator generator;
            constexpr double trajectory_duration = 3.0; // 3 seconds
            p_robot.trajectory = generator.generate(current_positions,
                                                    p_robot_state.ik_solution,
                                                    trajectory_duration);

            // Switch to TRAJECTORY mode
            p_robot.control_mode =
                renderer::RobotManager::ControlMode::TRAJECTORY;
            p_robot.trajectory_segment = 0;
            p_robot_state.trajectory_from_ik = true;

            std::cout << "🎯 Created trajectory to IK solution (duration: "
                      << trajectory_duration << "s)" << std::endl;
        }
        else
        {
            // IK failed to converge
            std::cout << "⚠️ IK failed to converge for target "
                      << (p_robot_state.target_pose_index + 1)
                      << " - Error: " << p_robot.ik_solver->poseError()
                      << std::endl;

            // Set failure flags to show popup
            p_robot_state.ik_failed = true;
            p_robot_state.show_failure_popup = true;
            p_robot_state.solution_computed = true;
        }
    }
}

// ----------------------------------------------------------------------------
void Controller::handleTrajectory(
    renderer::RobotManager::ControlledRobot& p_robot,
    double p_time,
    double p_dt) const
{
    auto& robot_state = m_robot_states[p_robot.name()];

    // If no trajectory is active, create a new one (for manual trajectory mode)
    if (!p_robot.trajectory)
    {
        // Only create automatic trajectories if not coming from IK
        if (robot_state.trajectory_from_ik)
        {
            // This shouldn't happen, but handle gracefully
            std::cout << "⚠️ No trajectory but trajectory_from_ik is set"
                      << std::endl;
            robot_state.trajectory_from_ik = false;
            p_robot.control_mode =
                renderer::RobotManager::ControlMode::INVERSE_KINEMATICS;
            return;
        }

        if (p_robot.trajectory_configs.empty())
        {
            std::cout << "🤖 Error: no trajectory configurations" << std::endl;
            return;
        }

        size_t start_idx =
            p_robot.trajectory_segment % p_robot.trajectory_configs.size();
        size_t goal_idx = (p_robot.trajectory_segment + 1) %
                          p_robot.trajectory_configs.size();

        std::cout << "🎯 Creating trajectory from config " << (start_idx + 1)
                  << " to config " << (goal_idx + 1) << std::endl;

        // Create trajectory generator
        robotik::JointSpaceGenerator generator;

        // Generate trajectory with 3 second duration
        double trajectory_duration = 3.0;
        p_robot.trajectory =
            generator.generate(p_robot.trajectory_configs[start_idx],
                               p_robot.trajectory_configs[goal_idx],
                               trajectory_duration);

        p_robot.trajectory_start_time = p_time;
    }

    // Evaluate trajectory at current time
    double t = p_time - p_robot.trajectory_start_time;

    if (t >= p_robot.trajectory->duration())
    {
        // Trajectory complete
        std::cout << "✅ Trajectory segment "
                  << (p_robot.trajectory_segment + 1) << " complete"
                  << std::endl;

        // Check if this trajectory was created by IK
        if (robot_state.trajectory_from_ik)
        {
            // Return to IK mode and move to next target
            std::cout << "🔄 Returning to INVERSE_KINEMATICS mode" << std::endl;
            p_robot.control_mode =
                renderer::RobotManager::ControlMode::INVERSE_KINEMATICS;
            robot_state.trajectory_from_ik = false;

            // Move to next IK target
            robot_state.target_pose_index++;
            if (robot_state.target_pose_index >= p_robot.ik_target_poses.size())
            {
                // Completed all targets, stop the cycle
                robot_state.target_pose_index = 0;
                robot_state.ik_cycle_complete = true;
                std::cout << "✅ All IK targets reached! Cycle complete."
                          << std::endl;
                std::cout
                    << "   (Switch to another mode or restart IK to continue)"
                    << std::endl;
            }
            robot_state.solution_computed = false; // Recompute for next target
        }
        else
        {
            // Manual trajectory mode: move to next segment
            p_robot.trajectory_segment++;
        }

        p_robot.trajectory.reset(); // Clear trajectory
        return;
    }

    // Get target position from trajectory
    auto trajectory_point = p_robot.trajectory->evaluate(t);

    // Apply with velocity limits
    p_robot.applyJointTargetsWithSpeedLimit(trajectory_point.position, p_dt);
}

// ----------------------------------------------------------------------------
void Controller::renderIKFailurePopup(std::string const& p_robot_name)
{
    auto* robot = m_robot_manager.getRobot(p_robot_name);
    if (robot == nullptr)
        return;

    auto& robot_state = m_robot_states[p_robot_name];

    // Only show popup if IK failed and flag is set
    if (!robot_state.show_failure_popup)
        return;

    // Open the popup
    ImGui::OpenPopup("IK Failure");

    // Center the popup
    ImVec2 center = ImGui::GetMainViewport()->GetCenter();
    ImGui::SetNextWindowPos(center, ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));

    if (ImGui::BeginPopupModal(
            "IK Failure", nullptr, ImGuiWindowFlags_AlwaysAutoResize))
    {
        ImGui::Text("Inverse Kinematics failed to converge for target %zu.",
                    robot_state.target_pose_index + 1);
        ImGui::Separator();
        ImGui::Text("What would you like to do?");
        ImGui::Spacing();

        // Button 1: Return to Home
        if (ImGui::Button("Return to Home", ImVec2(200, 0)))
        {
            std::cout << "🏠 Returning to home configuration" << std::endl;

            // Get current joint positions
            std::vector<double> current_positions;
            current_positions.reserve(robot->blueprint().numJoints());
            robot->blueprint().forEachJoint(
                [&current_positions](Joint const& joint, size_t /*index*/)
                { current_positions.push_back(joint.position()); });

            // Create trajectory to home configuration
            robotik::JointSpaceGenerator generator;
            constexpr double trajectory_duration = 3.0; // 3 seconds
            robot->trajectory =
                generator.generate(current_positions,
                                   robot_state.home_configuration,
                                   trajectory_duration);

            // Switch to TRAJECTORY mode
            robot->control_mode =
                renderer::RobotManager::ControlMode::TRAJECTORY;
            robot->trajectory_segment = 0;
            robot_state.trajectory_from_ik =
                true; // Will return to IK mode after

            // Close popup and reset flags
            robot_state.show_failure_popup = false;
            ImGui::CloseCurrentPopup();
        }

        ImGui::Spacing();

        // Button 2: Stay Immobile
        if (ImGui::Button("Stay Immobile", ImVec2(200, 0)))
        {
            std::cout << "🛑 Staying at current position, moving to next target"
                      << std::endl;

            // Move to next IK target
            robot_state.target_pose_index++;
            if (robot_state.target_pose_index >= robot->ik_target_poses.size())
            {
                robot_state.target_pose_index = 0;
            }
            robot_state.solution_computed = false; // Recompute for next target
            robot_state.ik_failed = false;

            // Close popup and reset flags
            robot_state.show_failure_popup = false;
            ImGui::CloseCurrentPopup();
        }

        ImGui::EndPopup();
    }
}

} // namespace robotik::application
