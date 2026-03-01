/**
 * @file ApplicationController.cpp
 * @brief Application control logic (Controller module of the MVC pattern).
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#include "ApplicationController.hpp"
#include "Robotik/Core/Robot/Blueprint/Node.hpp"
#include "Robotik/Core/Robot/Debug.hpp"
#include "Robotik/Core/Solvers/IKSolver.hpp"

#include <iostream>

namespace robotik::application
{

// ----------------------------------------------------------------------------
ApplicationController::ApplicationController(Configuration const& p_config)
    : m_config(p_config)
{
    m_robot_manager = std::make_unique<RobotManager>();
    m_teach_pendant = std::make_unique<TeachPendant>();
    m_ik_solver = std::make_unique<JacobianIKSolver>();
    m_camera_controller = std::make_unique<CameraController>(
        m_config.window_width, m_config.window_height);
    m_waypoint_manager = std::make_unique<WaypointManager>();
    m_trajectory_controller = std::make_unique<TrajectoryController>();
    m_behavior_tree_manager = std::make_unique<BehaviorTreeManager>();

    // Register robot actions once with the RobotManager (not per-robot)
    initializeBehaviorTree();
}

// ----------------------------------------------------------------------------
ControlledRobot*
ApplicationController::loadRobot(std::string const& p_urdf_file)
{
    // Load robot from URDF file
    auto* robot = m_robot_manager->loadRobot<ControlledRobot>(p_urdf_file);
    if (robot == nullptr)
    {
        m_error = "Failed to load robot from URDF: " + m_robot_manager->error();
        return nullptr;
    }

    std::cout << "Loaded robot from: " << p_urdf_file << std::endl;
    std::cout << robotik::debug::printRobot(*robot, true) << std::endl;

    // Set the robot configuration (home position, end effector selection,
    // camera target selection)
    if (!initializeRobot(*robot,
                         m_config.control_link,
                         m_config.home_position,
                         m_config.camera_target))
    {
        m_error = "Failed to initialize robot configuration";
        return nullptr;
    }

    return robot;
}

// ----------------------------------------------------------------------------
void ApplicationController::initializeBehaviorTree()
{
    // Register robot-specific actions with RobotManager (uses currentRobot())
    m_behavior_tree_manager->registerRobotActions(*m_robot_manager,
                                                  *m_teach_pendant,
                                                  *m_ik_solver,
                                                  *m_trajectory_controller);

    std::cout << "✅ Registered robot actions in behavior tree factory"
              << std::endl;
}

// ----------------------------------------------------------------------------
bool ApplicationController::initializeRobot(
    ControlledRobot& p_controlled_robot,
    std::string const& p_control_link,
    std::vector<double> const& p_joint_positions,
    std::string const& p_camera_target) const
{
    // Set initial joint positions from configuration or neutral position
    if (!p_joint_positions.empty())
    {
        p_controlled_robot.setJointPositions(p_joint_positions);
        std::cout << "🤖 Set initial joint values from configuration"
                  << std::endl;
    }
    else
    {
        p_controlled_robot.setNeutralPosition();
        std::cout << "🤖 Set neutral position" << std::endl;
    }

    // Store home position if provided
    if (!p_controlled_robot.setHomePosition(
            p_controlled_robot.states().joint_positions))
    {
        std::cerr << "⚠️ Home position was adjusted to fit joint limits"
                  << std::endl;
    }

    // Set camera target (always use root if not found)
    if (!p_camera_target.empty())
    {
        p_controlled_robot.camera_target = robotik::Node::find(
            p_controlled_robot.blueprint().root(), p_camera_target);
    }

    // Use root if target not found or not specified
    if (p_controlled_robot.camera_target == nullptr)
    {
        p_controlled_robot.camera_target =
            &p_controlled_robot.blueprint().root();
    }

    if (p_controlled_robot.camera_target != nullptr)
    {
        std::cout << "📷 Camera target: "
                  << p_controlled_robot.camera_target->name() << std::endl;
        p_controlled_robot.camera_tracking_enabled = true;
    }

    // Set end effector for the robot
    if (!p_control_link.empty())
    {
        p_controlled_robot.end_effector = robotik::Node::find(
            p_controlled_robot.blueprint().root(), p_control_link);
    }
    else
    {
        // Use first end effector if available
        if (auto const& end_effectors =
                p_controlled_robot.blueprint().endEffectors();
            !end_effectors.empty())
        {
            p_controlled_robot.end_effector = &end_effectors[0].get();
        }
    }

    // Robot shall have an end effector
    if (p_controlled_robot.end_effector != nullptr)
    {
        std::cout << "🤖 End effector set to: "
                  << p_controlled_robot.end_effector->name() << std::endl;
    }
    else
    {
        std::cerr << "⚠️ Error: no end effector found for robot: "
                  << p_controlled_robot.name() << std::endl;
        return false;
    }

    return true;
}

// ----------------------------------------------------------------------------
void ApplicationController::update(float const dt)
{
    // Camera tracking - use active robot
    auto* robot = getCurrentRobot();
    if (robot && robot->camera_tracking_enabled && robot->camera_target)
    {
        Eigen::Vector3f target_pos = robot->camera_target->worldTransform()
                                         .block<3, 1>(0, 3)
                                         .cast<float>();

        m_camera_controller->setTrackingEnabled(true);
        m_camera_controller->update(dt, &target_pos);
    }

    m_camera_controller->setTrackingEnabled(false);
    m_camera_controller->update(dt, nullptr);

    // Update behavior tree if playing
    if (m_behavior_tree_manager->isPlaying())
    {
        m_behavior_tree_manager->tick(dt);
    }
    else
    {
        auto const& robots = m_robot_manager->robots();
        for (auto const& [name, robot_ptr] : robots)
        {
            if (auto* controlled_robot =
                    dynamic_cast<ControlledRobot*>(robot_ptr.get());
                controlled_robot != nullptr &&
                controlled_robot->state == ControlledRobot::State::PLAYING)
            {
                controlled_robot->state = ControlledRobot::State::IDLE;
            }
        }
    }

    // Update trajectory controller (always update if playing, even if BT
    // stopped)
    if (m_trajectory_controller->isPlaying())
    {
        m_trajectory_controller->update(static_cast<double>(dt));
        // Apply current target to robot
        auto target = m_trajectory_controller->getCurrentTarget();
        if (!target.empty() && robot)
        {
            robot->setJointPositions(target);
        }
    }
}

// ----------------------------------------------------------------------------
bool ApplicationController::setEndEffector(std::string const& p_robot_name,
                                           std::string const& p_link_name) const
{
    auto* controlled_robot = getRobot(p_robot_name);
    if (controlled_robot == nullptr)
        return false;

    robotik::Node const* node =
        robotik::Node::find(controlled_robot->blueprint().root(), p_link_name);
    if (node == nullptr)
        return false;

    controlled_robot->end_effector = node;
    return true;
}

// ----------------------------------------------------------------------------
bool ApplicationController::setCameraTarget(
    std::string const& p_robot_name,
    std::string const& p_node_name) const
{
    auto* controlled_robot = getRobot(p_robot_name);
    if (controlled_robot == nullptr)
        return false;

    controlled_robot->camera_target =
        robotik::Node::find(controlled_robot->blueprint().root(), p_node_name);
    if (controlled_robot->camera_target == nullptr)
        return false;

    controlled_robot->camera_tracking_enabled = true;
    return true;
}

// ----------------------------------------------------------------------------
bool ApplicationController::setCartesianFrame(
    std::string const& p_robot_name,
    std::string const& p_node_name) const
{
    auto* controlled_robot = getRobot(p_robot_name);
    if (controlled_robot == nullptr)
        return false;

    // Empty node name means world frame (nullptr)
    if (p_node_name.empty())
    {
        controlled_robot->cartesian_frame = nullptr;
        return true;
    }

    controlled_robot->cartesian_frame =
        robotik::Node::find(controlled_robot->blueprint().root(), p_node_name);
    if (controlled_robot->cartesian_frame == nullptr)
        return false;

    return true;
}

// ----------------------------------------------------------------------------
ControlledRobot* ApplicationController::getCurrentRobot() const
{
    return m_robot_manager->currentRobot<ControlledRobot>();
}

// ----------------------------------------------------------------------------
ControlledRobot*
ApplicationController::getRobot(std::string const& p_robot_name) const
{
    return m_robot_manager->getRobot<ControlledRobot>(p_robot_name);
}

} // namespace robotik::application
