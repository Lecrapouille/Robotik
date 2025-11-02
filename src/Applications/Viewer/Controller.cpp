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
#include "Robotik/Core/Robot/Debug.hpp"
#include "Robotik/Core/Solvers/IKSolver.hpp"
#include "Robotik/Core/Solvers/Trajectory.hpp"

#include <iostream>

namespace robotik::application
{

// ----------------------------------------------------------------------------
Controller::Controller(renderer::RobotManager& p_robot_manager)
    : m_robot_manager(p_robot_manager)
{
}

// ----------------------------------------------------------------------------
bool Controller::setupRobots(Configuration const& p_config)
{
    m_robot_manager.clear();
    m_error.clear();

    // Load robots from URDF files
    for (auto const& urdf_file : p_config.urdf_files)
    {
        // Load robot from URDF file
        auto* robot = m_robot_manager.loadRobot(urdf_file);
        if (robot == nullptr)
        {
            m_error =
                "Failed to load robot from URDF: " + m_robot_manager.error() +
                "\n";
            return false;
        }

        std::cout << "Loaded robot from: " << urdf_file << std::endl;
        std::cout << robotik::debug::printRobot(*robot, true) << std::endl;

        if (!initializeRobot(*robot, p_config))
            return false;
    }

    return true;
}

// ----------------------------------------------------------------------------
bool Controller::initializeRobot(
    renderer::RobotManager::ControlledRobot& p_robot,
    Configuration const& p_config)
{
    // Set initial joint values from configuration. If no configuration is
    // provided, set neutral position.
    if (auto const& joint_positions = p_config.joint_positions;
        !joint_positions.empty())
    {
        p_robot.setJointPositions(joint_positions);
        std::cout << "🤖 Set initial joint values from configuration"
                  << std::endl;
    }
    else
    {
        p_robot.setNeutralPosition();
        std::cout << "🤖 Set neutral position" << std::endl;
    }

    // Set camera target to the specified robot element.
    if (setCameraTarget(
            p_robot, p_config.camera_target, p_config.use_root_if_not_found))
    {
        std::cout << "📷 Camera target: " << p_robot.camera_target->name()
                  << std::endl;
        p_robot.camera_tracking_enabled = true;
    }
    else
    {
        m_error = "📷 Failed to set camera target: " + p_config.camera_target;
        return false;
    }

    // Set control link (end effector) for inverse kinematics.
    // If no joint name is provided, use first end effector.
    // If joint name is provided, use it.
    // If joint name is not found, use base_link.
    p_robot.control_link = nullptr;
    if (!p_config.control_link.empty())
    {
        // Try to find the link by name.
        p_robot.control_link =
            Node::find(p_robot.blueprint().root(), p_config.control_link);

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
        m_error =
            "🤖 Error: control link not found for robot: " + p_robot.name();
        return false;
    }

    // Compute IK target poses if control link is set
    computeIKTargetPoses(p_robot);

    // Compute trajectory configurations
    computeTrajectoryConfigs(p_robot);

    // Initialize robot state
    m_robot_states[p_robot.name()] = RobotState{};

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

    // Recompute IK targets if not already computed.
    if (p_mode == renderer::RobotManager::ControlMode::INVERSE_KINEMATICS &&
        (!robot->ik_target_poses.empty()))
    {
        computeIKTargetPoses(*robot);
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
bool Controller::setCameraTarget(
    renderer::RobotManager::ControlledRobot& p_robot,
    std::string const& p_element_name,
    bool p_use_root_if_not_found) const
{
    p_robot.camera_target = nullptr;

    if (p_element_name.empty())
    {
        if (p_use_root_if_not_found)
        {
            p_robot.camera_target = &p_robot.blueprint().root();
        }
        else if (auto const& end_effectors = p_robot.blueprint().endEffectors();
                 !end_effectors.empty())
        {
            p_robot.camera_target = &end_effectors[0].get();
        }
    }
    else
    {
        p_robot.camera_target =
            Node::find(p_robot.blueprint().root(), p_element_name);
        if (!p_robot.camera_target && p_use_root_if_not_found)
        {
            p_robot.camera_target = &p_robot.blueprint().root();
        }
    }

    return p_robot.camera_target != nullptr;
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
    double p_time)
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
    double p_dt,
    RobotState& p_robot_state)
{
    if (p_robot.control_link == nullptr || p_robot.ik_solver == nullptr)
    {
        m_error = "🤖 Error: control link or solver not set";
        return;
    }

    // Get current target pose based on state
    Pose const& target_pose =
        p_robot.ik_target_poses[p_robot_state.target_pose_index];

    // Solve IK for current target
    if (bool solved = p_robot.ik_solver->solve(
            p_robot, *p_robot.control_link, target_pose);
        solved)
    {
        std::cout << "🎯 Reached target "
                  << (p_robot_state.target_pose_index + 1)
                  << " - Error: " << p_robot.ik_solver->poseError()
                  << " - Iterations: " << p_robot.ik_solver->numIterations()
                  << std::endl;

        // Transition to next state
        p_robot_state.target_pose_index++;
        if (p_robot_state.target_pose_index >= p_robot.ik_target_poses.size())
        {
            p_robot_state.target_pose_index = 0;
        }
    }
    else
    {
        // Only log failures occasionally to avoid spam
        static size_t failure_count = 0;
        if (++failure_count % 100 == 0)
        {
            std::cout << "⚠️ IK solving in progress - Pose error: "
                      << p_robot.ik_solver->poseError() << std::endl;
        }
    }

    // Apply with velocity limits
    p_robot.applyJointTargetsWithSpeedLimit(p_robot.state().joint_positions,
                                            p_dt);
}

// ----------------------------------------------------------------------------
void Controller::handleTrajectory(
    renderer::RobotManager::ControlledRobot& p_robot,
    double p_time,
    double p_dt)
{
    if (p_robot.trajectory_configs.empty())
    {
        m_error = "🤖 Error: no trajectory configurations";
        return;
    }

    // If no trajectory is active, create a new one
    if (!p_robot.trajectory)
    {
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
        // Trajectory complete, move to next segment
        std::cout << "✅ Trajectory segment "
                  << (p_robot.trajectory_segment + 1) << " complete"
                  << std::endl;

        p_robot.trajectory_segment++;
        p_robot.trajectory.reset(); // Will create new trajectory on next update
        return;
    }

    // Get target position from trajectory
    auto trajectory_point = p_robot.trajectory->evaluate(t);

    // Apply with velocity limits
    p_robot.applyJointTargetsWithSpeedLimit(trajectory_point.position, p_dt);
}

} // namespace robotik::application
