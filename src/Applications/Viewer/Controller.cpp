/**
 * @file Controller.cpp
 * @brief Controller for robot control logic implementation.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#include "Controller.hpp"
#include "Robotik/Core/Robot/Blueprint/Node.hpp"
#include "Robotik/Core/Solvers/IKSolver.hpp"

#include <iostream>

namespace robotik::application
{

// ----------------------------------------------------------------------------
Controller::Controller(renderer::RobotManager& p_robot_manager)
    : m_robot_manager(p_robot_manager),
      m_teach_pendant(std::make_unique<robotik::TeachPendant>()),
      m_ik_solver(std::make_unique<robotik::JacobianIKSolver>())
{
}

// ----------------------------------------------------------------------------
bool Controller::initializeRobot(ControlledRobot* p_controlled_robot,
                                 std::string const& p_control_link,
                                 std::vector<double> const& p_joint_positions,
                                 std::string const& p_camera_target)
{
    // Set initial joint positions from configuration or neutral position
    if (!p_joint_positions.empty())
    {
        p_controlled_robot->setJointPositions(p_joint_positions);
        std::cout << "🤖 Set initial joint values from configuration"
                  << std::endl;
    }
    else
    {
        p_controlled_robot->setNeutralPosition();
        std::cout << "🤖 Set neutral position" << std::endl;
    }

    // Store home position
    p_controlled_robot->setHomePosition(
        p_controlled_robot->states().joint_positions);

    // Set end effector for the robot
    if (!p_control_link.empty())
    {
        p_controlled_robot->end_effector = robotik::Node::find(
            p_controlled_robot->blueprint().root(), p_control_link);
    }
    else
    {
        // Use first end effector if available
        if (auto const& end_effectors =
                p_controlled_robot->blueprint().endEffectors();
            !end_effectors.empty())
        {
            p_controlled_robot->end_effector = &end_effectors[0].get();
        }
    }

    if (p_controlled_robot->end_effector != nullptr)
    {
        std::cout << "🤖 End effector set to: "
                  << p_controlled_robot->end_effector->name() << std::endl;
    }
    else
    {
        std::cout << "⚠️ Warning: no end effector found for robot: "
                  << p_controlled_robot->name() << std::endl;
    }

    // Set camera target (always use root if not found)
    if (!p_camera_target.empty())
    {
        p_controlled_robot->camera_target = robotik::Node::find(
            p_controlled_robot->blueprint().root(), p_camera_target);
    }

    // Use root if target not found or not specified
    if (p_controlled_robot->camera_target == nullptr)
    {
        p_controlled_robot->camera_target =
            &p_controlled_robot->blueprint().root();
    }

    if (p_controlled_robot->camera_target != nullptr)
    {
        std::cout << "📷 Camera target: "
                  << p_controlled_robot->camera_target->name() << std::endl;
        p_controlled_robot->camera_tracking_enabled = true;
    }

    return true;
}

// ----------------------------------------------------------------------------
void Controller::update(double p_dt)
{
    for (auto const& [robot_name, robot_ptr] : m_robot_manager.robots())
    {
        auto* controlled_robot =
            m_robot_manager.getRobot<ControlledRobot>(robot_name);
        if (controlled_robot &&
            controlled_robot->state == ControlledRobot::State::PLAYING)
        {
            m_teach_pendant->setRobot(*controlled_robot);
            m_teach_pendant->update(p_dt);
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

    controlled_robot->end_effector = node;
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
    return m_robot_manager.getRobot<ControlledRobot>(p_robot_name);
}

// ----------------------------------------------------------------------------
robotik::TeachPendant* Controller::getTeachPendant()
{
    return m_teach_pendant.get();
}

} // namespace robotik::application
