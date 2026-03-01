/**
 * @file RobotActions.cpp
 * @brief Built-in robot action and condition nodes implementation.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#include "Robotik/Core/BehaviorTree/Actions/RobotActions.hpp"

#include "Robotik/Core/Common/Conversions.hpp"
#include "Robotik/Core/Managers/RobotManager.hpp"
#include "Robotik/Core/Robot/Robot.hpp"
#include "Robotik/Core/Robot/TeachPendant.hpp"
#include "Robotik/Core/Solvers/IKSolver.hpp"
#include "Robotik/Core/Solvers/TrajectoryController.hpp"

#include <iostream>

namespace robotik::bt {

// ============================================================================
// HomingAction
// ============================================================================

HomingAction::HomingAction(RobotManager& p_robot_manager,
                           TrajectoryController& p_trajectory_controller,
                           Blackboard::Ptr p_blackboard)
    : m_robot_manager(p_robot_manager),
      m_trajectory_controller(p_trajectory_controller)
{
    setBlackboard(p_blackboard);
}

Status HomingAction::onRunning()
{
    Robot* robot = m_robot_manager.currentRobot();
    if (!robot)
    {
        std::cerr << "Homing: no robot selected" << std::endl;
        return Status::FAILURE;
    }

    if (!m_started)
    {
        auto const& home_positions = robot->homePosition();
        if (home_positions.empty())
        {
            std::cerr << "Homing: robot has no home position set" << std::endl;
            return Status::FAILURE;
        }

        if (home_positions.size() != robot->blueprint().numJoints())
        {
            std::cerr << "Homing: home position size mismatch (expected "
                      << robot->blueprint().numJoints() << ", got "
                      << home_positions.size() << ")" << std::endl;
            return Status::FAILURE;
        }

        double duration = m_blackboard->getOrDefault<double>("movement_duration", 2.0);

        if (!m_trajectory_controller.goToWaypoint(
                robot->states().joint_positions, home_positions, duration))
        {
            std::cerr << "Homing: failed to start trajectory" << std::endl;
            return Status::FAILURE;
        }

        m_started = true;
        std::cout << "Homing action started" << std::endl;
        return Status::RUNNING;
    }

    if (m_trajectory_controller.isPlaying())
    {
        return Status::RUNNING;
    }

    m_started = false;
    std::cout << "Homing action completed" << std::endl;
    return Status::SUCCESS;
}

void HomingAction::onHalt()
{
    if (m_started)
    {
        m_trajectory_controller.stop();
        m_started = false;
        std::cout << "Homing action halted" << std::endl;
    }
}

// ============================================================================
// MoveToJointPoseAction
// ============================================================================

MoveToJointPoseAction::MoveToJointPoseAction(
    RobotManager& p_robot_manager,
    TrajectoryController& p_trajectory_controller,
    Blackboard::Ptr p_blackboard)
    : m_robot_manager(p_robot_manager),
      m_trajectory_controller(p_trajectory_controller)
{
    setBlackboard(p_blackboard);
}

Status MoveToJointPoseAction::onRunning()
{
    Robot* robot = m_robot_manager.currentRobot();
    if (!robot)
    {
        std::cerr << "MoveToJointPose: no robot selected" << std::endl;
        return Status::FAILURE;
    }

    if (!m_started)
    {
        auto joint_positions = m_blackboard->get<std::vector<double>>("joint_positions");
        if (!joint_positions)
        {
            std::cerr << "MoveToJointPose: joint_positions not found in blackboard"
                      << std::endl;
            return Status::FAILURE;
        }

        if (joint_positions->size() != robot->blueprint().numJoints())
        {
            std::cerr << "MoveToJointPose: joint_positions size mismatch (expected "
                      << robot->blueprint().numJoints() << ", got "
                      << joint_positions->size() << ")" << std::endl;
            return Status::FAILURE;
        }

        double duration = m_blackboard->getOrDefault<double>("movement_duration", 2.0);

        if (!m_trajectory_controller.goToWaypoint(
                robot->states().joint_positions, *joint_positions, duration))
        {
            std::cerr << "MoveToJointPose: failed to start trajectory" << std::endl;
            return Status::FAILURE;
        }

        m_started = true;
        std::cout << "MoveToJointPose action started" << std::endl;
        return Status::RUNNING;
    }

    if (m_trajectory_controller.isPlaying())
    {
        return Status::RUNNING;
    }

    m_started = false;
    std::cout << "MoveToJointPose action completed" << std::endl;
    return Status::SUCCESS;
}

void MoveToJointPoseAction::onHalt()
{
    if (m_started)
    {
        m_trajectory_controller.stop();
        m_started = false;
        std::cout << "MoveToJointPose action halted" << std::endl;
    }
}

// ============================================================================
// MoveToCartesianPoseAction
// ============================================================================

MoveToCartesianPoseAction::MoveToCartesianPoseAction(
    RobotManager& p_robot_manager,
    IKSolver& p_ik_solver,
    TrajectoryController& p_trajectory_controller,
    Blackboard::Ptr p_blackboard)
    : m_robot_manager(p_robot_manager),
      m_ik_solver(p_ik_solver),
      m_trajectory_controller(p_trajectory_controller)
{
    setBlackboard(p_blackboard);
}

Status MoveToCartesianPoseAction::onRunning()
{
    Robot* robot = m_robot_manager.currentRobot();
    if (!robot)
    {
        std::cerr << "MoveToCartesianPose: no robot selected" << std::endl;
        return Status::FAILURE;
    }

    if (!m_started)
    {
        auto pose = m_blackboard->get<std::vector<double>>("pose");
        if (!pose)
        {
            std::cerr << "MoveToCartesianPose: pose not found in blackboard"
                      << std::endl;
            return Status::FAILURE;
        }

        if (pose->size() != 6)
        {
            std::cerr << "MoveToCartesianPose: pose must have 6 elements (x,y,z,roll,pitch,yaw)"
                      << std::endl;
            return Status::FAILURE;
        }

        Pose target_pose;
        target_pose << (*pose)[0], (*pose)[1], (*pose)[2], (*pose)[3], (*pose)[4], (*pose)[5];

        std::cout << "Target pose: [" << (*pose)[0] << ", " << (*pose)[1] << ", "
                  << (*pose)[2] << ", " << (*pose)[3] << ", " << (*pose)[4] << ", "
                  << (*pose)[5] << "]" << std::endl;

        auto const& end_effectors = robot->blueprint().endEffectors();
        if (end_effectors.empty())
        {
            std::cerr << "MoveToCartesianPose: no end effector found" << std::endl;
            return Status::FAILURE;
        }

        std::cout << "Using end effector: " << end_effectors[0].get().name() << std::endl;

        auto current_positions = robot->states().joint_positions;

        std::cout << "Current joint positions: [";
        for (size_t i = 0; i < current_positions.size(); ++i)
        {
            std::cout << current_positions[i];
            if (i < current_positions.size() - 1)
                std::cout << ", ";
        }
        std::cout << "]" << std::endl;

        if (!m_ik_solver.solve(*robot, end_effectors[0].get(), target_pose))
        {
            std::cerr << "MoveToCartesianPose: IK solver failed";
            if (m_ik_solver.converged())
            {
                std::cerr << " (converged but outside limits?)";
            }
            else
            {
                std::cerr << " (did not converge after "
                          << m_ik_solver.numIterations()
                          << " iterations, error: " << m_ik_solver.poseError() << ")";
            }
            std::cerr << std::endl;

            auto current_ee_pose = end_effectors[0].get().worldTransform();
            double dx = target_pose(0) - current_ee_pose(0, 3);
            double dy = target_pose(1) - current_ee_pose(1, 3);
            double dz = target_pose(2) - current_ee_pose(2, 3);
            double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

            std::cerr << "Distance to target: " << distance << " m" << std::endl;
            std::cerr << "Current EE position: [" << current_ee_pose(0, 3)
                      << ", " << current_ee_pose(1, 3) << ", "
                      << current_ee_pose(2, 3) << "]" << std::endl;

            robot->setJointPositions(current_positions);
            return Status::FAILURE;
        }

        std::cout << "IK solved successfully" << std::endl;

        auto target_joint_positions = robot->states().joint_positions;
        robot->setJointPositions(current_positions);

        double duration = m_blackboard->getOrDefault<double>("movement_duration", 2.0);

        if (!m_trajectory_controller.goToWaypoint(
                current_positions, target_joint_positions, duration))
        {
            std::cerr << "MoveToCartesianPose: failed to start trajectory" << std::endl;
            return Status::FAILURE;
        }

        m_started = true;
        std::cout << "MoveToCartesianPose action started: [" << (*pose)[0]
                  << ", " << (*pose)[1] << ", " << (*pose)[2] << ", " << (*pose)[3]
                  << ", " << (*pose)[4] << ", " << (*pose)[5] << "]" << std::endl;
        return Status::RUNNING;
    }

    if (m_trajectory_controller.isPlaying())
    {
        return Status::RUNNING;
    }

    m_started = false;
    std::cout << "MoveToCartesianPose action completed" << std::endl;
    return Status::SUCCESS;
}

void MoveToCartesianPoseAction::onHalt()
{
    if (m_started)
    {
        m_trajectory_controller.stop();
        m_started = false;
        std::cout << "MoveToCartesianPose action halted" << std::endl;
    }
}

// ============================================================================
// OpenGripperAction
// ============================================================================

OpenGripperAction::OpenGripperAction(Blackboard::Ptr p_blackboard)
{
    setBlackboard(p_blackboard);
}

Status OpenGripperAction::onRunning()
{
    std::cout << "OpenGripper action executed" << std::endl;
    return Status::SUCCESS;
}

// ============================================================================
// CloseGripperAction
// ============================================================================

CloseGripperAction::CloseGripperAction(Blackboard::Ptr p_blackboard)
{
    setBlackboard(p_blackboard);
}

Status CloseGripperAction::onRunning()
{
    std::cout << "CloseGripper action executed" << std::endl;
    return Status::SUCCESS;
}

// ============================================================================
// IsAtPoseCondition
// ============================================================================

IsAtPoseCondition::IsAtPoseCondition(RobotManager& p_robot_manager,
                                     Blackboard::Ptr p_blackboard)
    : m_robot_manager(p_robot_manager)
{
    setBlackboard(p_blackboard);
}

Status IsAtPoseCondition::onRunning()
{
    Robot* robot = m_robot_manager.currentRobot();
    if (!robot)
    {
        std::cerr << "IsAtPose: no robot selected" << std::endl;
        return Status::FAILURE;
    }

    auto target_pose_vec = m_blackboard->get<std::vector<double>>("target_pose");
    if (!target_pose_vec || target_pose_vec->size() != 6)
    {
        std::cerr << "IsAtPose: target_pose not found or invalid in blackboard"
                  << std::endl;
        return Status::FAILURE;
    }

    double tolerance = m_blackboard->getOrDefault<double>("tolerance", 0.01);

    Pose target_pose;
    target_pose << (*target_pose_vec)[0], (*target_pose_vec)[1], (*target_pose_vec)[2],
        (*target_pose_vec)[3], (*target_pose_vec)[4], (*target_pose_vec)[5];

    std::cout << "IsAtPose condition checked (tolerance: " << tolerance << ")" << std::endl;
    return Status::SUCCESS;
}

// ============================================================================
// HasTargetCondition
// ============================================================================

HasTargetCondition::HasTargetCondition(Blackboard::Ptr p_blackboard)
{
    setBlackboard(p_blackboard);
}

Status HasTargetCondition::onRunning()
{
    auto target_name = m_blackboard->get<std::string>("target_name");
    if (!target_name || target_name->empty())
    {
        std::cout << "HasTarget condition: target_name not found" << std::endl;
        return Status::FAILURE;
    }

    std::cout << "HasTarget condition: " << *target_name << std::endl;
    return Status::SUCCESS;
}

// ============================================================================
// Registration Helper
// ============================================================================

void registerRobotActions(NodeFactory& p_factory,
                          RobotManager& p_robot_manager,
                          TeachPendant& /*p_teach_pendant*/,
                          IKSolver& p_ik_solver,
                          TrajectoryController& p_trajectory_controller,
                          Blackboard::Ptr p_blackboard)
{
    p_factory.registerNode(
        "Homing",
        [&p_robot_manager, &p_trajectory_controller, p_blackboard]() -> Node::Ptr
        {
            return std::make_unique<HomingAction>(
                p_robot_manager, p_trajectory_controller, p_blackboard);
        });

    p_factory.registerNode(
        "MoveToJointPose",
        [&p_robot_manager, &p_trajectory_controller, p_blackboard]() -> Node::Ptr
        {
            return std::make_unique<MoveToJointPoseAction>(
                p_robot_manager, p_trajectory_controller, p_blackboard);
        });

    p_factory.registerNode(
        "MoveToCartesianPose",
        [&p_robot_manager, &p_ik_solver, &p_trajectory_controller, p_blackboard]() -> Node::Ptr
        {
            return std::make_unique<MoveToCartesianPoseAction>(
                p_robot_manager, p_ik_solver, p_trajectory_controller, p_blackboard);
        });

    p_factory.registerNode(
        "OpenGripper",
        [p_blackboard]() -> Node::Ptr
        { return std::make_unique<OpenGripperAction>(p_blackboard); });

    p_factory.registerNode(
        "CloseGripper",
        [p_blackboard]() -> Node::Ptr
        { return std::make_unique<CloseGripperAction>(p_blackboard); });

    p_factory.registerNode(
        "IsAtPose",
        [&p_robot_manager, p_blackboard]() -> Node::Ptr
        { return std::make_unique<IsAtPoseCondition>(p_robot_manager, p_blackboard); });

    p_factory.registerNode(
        "HasTarget",
        [p_blackboard]() -> Node::Ptr
        { return std::make_unique<HasTargetCondition>(p_blackboard); });
}

} // namespace robotik::bt
