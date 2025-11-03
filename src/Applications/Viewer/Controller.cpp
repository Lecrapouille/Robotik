/**
 * @file Controller.cpp
 * @brief Controller for robot control logic implementation.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#include "Controller.hpp"
#include "Robotik/Core/Robot/Blueprint/Joint.hpp"
#include "Robotik/Core/Robot/Blueprint/Node.hpp"
#include "Robotik/Core/Solvers/IKSolver.hpp"

#include <iostream>

namespace robotik::application
{

// ----------------------------------------------------------------------------
Controller::Controller(renderer::RobotManager& p_robot_manager)
    : m_robot_manager(p_robot_manager)
{
}

// ----------------------------------------------------------------------------
bool Controller::initializeRobot(std::string const& p_robot_name,
                                 robotik::Blueprint&& p_robot_blueprint,
                                 std::string const& p_control_link,
                                 std::vector<double> const& p_joint_positions,
                                 std::string const& p_camera_target)
{
    // Create ControlledRobot instance
    auto controlled_robot = std::make_unique<ControlledRobot>(
        p_robot_name, std::move(p_robot_blueprint));

    // Set initial joint positions from configuration or neutral position
    if (!p_joint_positions.empty())
    {
        controlled_robot->setJointPositions(p_joint_positions);
        std::cout << "🤖 Set initial joint values from configuration"
                  << std::endl;
    }
    else
    {
        controlled_robot->setNeutralPosition();
        std::cout << "🤖 Set neutral position" << std::endl;
    }

    // Create teach pendant for the robot
    controlled_robot->teach_pendant =
        std::make_unique<robotik::TeachPendant>(*controlled_robot);

    // Set IK solver
    controlled_robot->teach_pendant->setIKSolver(
        std::make_unique<robotik::JacobianIKSolver>());

    // Set end effector for teach pendant
    robotik::Node const* end_effector = nullptr;
    if (!p_control_link.empty())
    {
        end_effector = robotik::Node::find(controlled_robot->blueprint().root(),
                                           p_control_link);
    }
    else
    {
        // Use first end effector if available
        if (auto const& end_effectors =
                controlled_robot->blueprint().endEffectors();
            !end_effectors.empty())
        {
            end_effector = &end_effectors[0].get();
        }
    }

    if (end_effector != nullptr)
    {
        controlled_robot->teach_pendant->setEndEffector(*end_effector);
        std::cout << "🤖 End effector set to: " << end_effector->name()
                  << std::endl;
    }
    else
    {
        std::cout << "⚠️ Warning: no end effector found for robot: "
                  << p_robot_name << std::endl;
    }

    // Set camera target (always use root if not found)
    if (!p_camera_target.empty())
    {
        controlled_robot->camera_target = robotik::Node::find(
            controlled_robot->blueprint().root(), p_camera_target);
    }

    // Use root if target not found or not specified
    if (controlled_robot->camera_target == nullptr)
    {
        controlled_robot->camera_target = &controlled_robot->blueprint().root();
    }

    if (controlled_robot->camera_target != nullptr)
    {
        std::cout << "📷 Camera target: "
                  << controlled_robot->camera_target->name() << std::endl;
        controlled_robot->camera_tracking_enabled = true;
    }

    // Store the controlled robot
    m_controlled_robots[p_robot_name] = std::move(controlled_robot);

    return true;
}

// ----------------------------------------------------------------------------
void Controller::update(double p_elapsed_time, double p_dt)
{
    for (auto& [robot_name, controlled_robot] : m_controlled_robots)
    {
        if (controlled_robot && controlled_robot->teach_pendant)
        {
            controlled_robot->teach_pendant->updateRobotState(p_dt);
        }
    }
}

// ----------------------------------------------------------------------------
bool Controller::setEndEffector(std::string const& p_robot_name,
                                std::string const& p_link_name)
{
    auto* controlled_robot = getControlledRobot(p_robot_name);
    if (controlled_robot == nullptr)
        return false;

    robotik::Node const* node =
        robotik::Node::find(controlled_robot->blueprint().root(), p_link_name);
    if (node == nullptr)
        return false;

    if (controlled_robot->teach_pendant)
    {
        controlled_robot->teach_pendant->setEndEffector(*node);
    }

    return true;
}

// ----------------------------------------------------------------------------
bool Controller::setCameraTarget(std::string const& p_robot_name,
                                 std::string const& p_node_name)
{
    auto* controlled_robot = getControlledRobot(p_robot_name);
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
ControlledRobot* Controller::getControlledRobot(std::string const& p_robot_name)
{
    auto it = m_controlled_robots.find(p_robot_name);
    if (it == m_controlled_robots.end())
        return nullptr;
    return it->second.get();
}

// ----------------------------------------------------------------------------
robotik::TeachPendant*
Controller::getTeachPendant(std::string const& p_robot_name)
{
    auto* controlled_robot = getControlledRobot(p_robot_name);
    if (controlled_robot == nullptr)
        return nullptr;
    return controlled_robot->teach_pendant.get();
}

} // namespace robotik::application
