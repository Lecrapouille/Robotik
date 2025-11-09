/**
 * @file RobotController.cpp
 * @brief Robot controller for robot control logic implementation.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#include "RobotController.hpp"
#include "Robotik/Core/Robot/Blueprint/Node.hpp"
#include "Robotik/Core/Solvers/IKSolver.hpp"

#include <iostream>

namespace robotik::application
{

// ----------------------------------------------------------------------------
RobotController::RobotController(Configuration const& p_config)
{
    m_robot_manager = std::make_unique<RobotManager>();
    m_teach_pendant = std::make_unique<robotik::TeachPendant>();
    m_ik_solver = std::make_unique<robotik::JacobianIKSolver>();
    camera_model = std::make_unique<CameraViewModel>(p_config.window_width,
                                                     p_config.window_height);
    waypoint_manager = std::make_unique<WaypointManager>();
    trajectory_controller = std::make_unique<TrajectoryController>();
}

// ----------------------------------------------------------------------------
bool RobotController::initializeRobot(
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

    // Store home position
    if (!p_controlled_robot.setHomePosition(
            p_controlled_robot.states().joint_positions))
    {
        std::cout << "⚠️ Home position was adjusted to fit joint limits"
                  << std::endl;
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

    if (p_controlled_robot.end_effector != nullptr)
    {
        std::cout << "🤖 End effector set to: "
                  << p_controlled_robot.end_effector->name() << std::endl;
    }
    else
    {
        std::cout << "⚠️ Warning: no end effector found for robot: "
                  << p_controlled_robot.name() << std::endl;
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

    return true;
}

// ----------------------------------------------------------------------------
void RobotController::update(double p_dt)
{
    // Trajectory control is now handled by MainApplication via
    // TrajectoryController This method can be used for other controller logic
    // if needed
    (void)p_dt; // Unused for now
}

// ----------------------------------------------------------------------------
bool RobotController::setEndEffector(std::string const& p_robot_name,
                                     std::string const& p_link_name)
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
bool RobotController::setCameraTarget(std::string const& p_robot_name,
                                      std::string const& p_node_name)
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
bool RobotController::setCartesianFrame(std::string const& p_robot_name,
                                        std::string const& p_node_name)
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
ControlledRobot* RobotController::getCurrentRobot() const
{
    return m_robot_manager->currentRobot<ControlledRobot>();
}

// ----------------------------------------------------------------------------
ControlledRobot*
RobotController::getRobot(std::string const& p_robot_name) const
{
    return m_robot_manager->getRobot<ControlledRobot>(p_robot_name);
}

} // namespace robotik::application
