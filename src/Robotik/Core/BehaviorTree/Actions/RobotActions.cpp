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
#include "Robotik/Core/Robot/Robot.hpp"
#include "Robotik/Core/Robot/TeachPendant.hpp"
#include "Robotik/Core/Solvers/IKSolver.hpp"
#include "Robotik/Core/Solvers/TrajectoryController.hpp"

#include <iostream>

namespace robotik
{

// ============================================================================
// HomingAction
// ============================================================================

HomingAction::HomingAction(Robot& p_robot,
                           TrajectoryController& p_trajectory_controller,
                           bt::Blackboard::Ptr p_blackboard)
    : bt::Leaf(p_blackboard),
      m_robot(p_robot),
      m_trajectory_controller(p_trajectory_controller)
{
}

bt::Status HomingAction::onRunning()
{
    // First call: start the trajectory to home position
    if (!m_started)
    {
        // Get home position from robot
        auto const& home_positions = m_robot.homePosition();
        if (home_positions.empty())
        {
            std::cerr << "❌ Homing: robot has no home position set"
                      << std::endl;
            return bt::Status::FAILURE;
        }

        // Validate size
        if (home_positions.size() != m_robot.blueprint().numJoints())
        {
            std::cerr << "❌ Homing: home position size mismatch (expected "
                      << m_robot.blueprint().numJoints() << ", got "
                      << home_positions.size() << ")" << std::endl;
            return bt::Status::FAILURE;
        }

        // Get duration from blackboard or use default
        auto [duration, duration_found] =
            m_blackboard->get<double>("movement_duration");
        if (!duration_found)
        {
            duration = 2.0; // Default 2 seconds
        }

        // Start trajectory to home position
        if (!m_trajectory_controller.goToWaypoint(
                m_robot.states().joint_positions, home_positions, duration))
        {
            std::cerr << "❌ Homing: failed to start trajectory" << std::endl;
            return bt::Status::FAILURE;
        }

        m_started = true;
        std::cout << "🏠 Homing action started" << std::endl;
        return bt::Status::RUNNING;
    }

    // Subsequent calls: check if trajectory is finished
    if (m_trajectory_controller.isPlaying())
    {
        return bt::Status::RUNNING;
    }

    // Trajectory finished
    m_started = false;
    std::cout << "✅ Homing action completed" << std::endl;
    return bt::Status::SUCCESS;
}

void HomingAction::onHalt()
{
    if (m_started)
    {
        m_trajectory_controller.stop();
        m_started = false;
        std::cout << "⏹️ Homing action halted" << std::endl;
    }
}

// ============================================================================
// MoveToJointPoseAction
// ============================================================================

MoveToJointPoseAction::MoveToJointPoseAction(
    Robot& p_robot,
    TrajectoryController& p_trajectory_controller,
    bt::Blackboard::Ptr p_blackboard)
    : bt::Leaf(p_blackboard),
      m_robot(p_robot),
      m_trajectory_controller(p_trajectory_controller)
{
}

bt::Status MoveToJointPoseAction::onRunning()
{
    // First call: start the trajectory
    if (!m_started)
    {
        // Get joint positions from blackboard
        auto [joint_positions, found] =
            m_blackboard->get<std::vector<double>>("joint_positions");
        if (!found)
        {
            std::cerr
                << "❌ MoveToJointPose: joint_positions not found in blackboard"
                << std::endl;
            return bt::Status::FAILURE;
        }

        // Validate size
        if (joint_positions.size() != m_robot.blueprint().numJoints())
        {
            std::cerr << "❌ MoveToJointPose: joint_positions size mismatch "
                         "(expected "
                      << m_robot.blueprint().numJoints() << ", got "
                      << joint_positions.size() << ")" << std::endl;
            return bt::Status::FAILURE;
        }

        // Get duration from blackboard or use default
        auto [duration, duration_found] =
            m_blackboard->get<double>("movement_duration");
        if (!duration_found)
        {
            duration = 2.0; // Default 2 seconds
        }

        // Start trajectory
        if (!m_trajectory_controller.goToWaypoint(
                m_robot.states().joint_positions, joint_positions, duration))
        {
            std::cerr << "❌ MoveToJointPose: failed to start trajectory"
                      << std::endl;
            return bt::Status::FAILURE;
        }

        m_started = true;
        std::cout << "🤖 MoveToJointPose action started" << std::endl;
        return bt::Status::RUNNING;
    }

    // Subsequent calls: check if trajectory is finished
    if (m_trajectory_controller.isPlaying())
    {
        return bt::Status::RUNNING;
    }

    // Trajectory finished
    m_started = false;
    std::cout << "✅ MoveToJointPose action completed" << std::endl;
    return bt::Status::SUCCESS;
}

void MoveToJointPoseAction::onHalt()
{
    if (m_started)
    {
        m_trajectory_controller.stop();
        m_started = false;
        std::cout << "⏹️ MoveToJointPose action halted" << std::endl;
    }
}

// ============================================================================
// MoveToCartesianPoseAction
// ============================================================================

MoveToCartesianPoseAction::MoveToCartesianPoseAction(
    Robot& p_robot,
    IKSolver& p_ik_solver,
    TrajectoryController& p_trajectory_controller,
    bt::Blackboard::Ptr p_blackboard)
    : bt::Leaf(p_blackboard),
      m_robot(p_robot),
      m_ik_solver(p_ik_solver),
      m_trajectory_controller(p_trajectory_controller)
{
}

bt::Status MoveToCartesianPoseAction::onRunning()
{
    // First call: compute IK and start trajectory
    if (!m_started)
    {
        // Get pose from blackboard
        auto [pose, found] = m_blackboard->get<std::vector<double>>("pose");
        if (!found)
        {
            std::cerr << "❌ MoveToCartesianPose: pose not found in blackboard"
                      << std::endl;
            return bt::Status::FAILURE;
        }

        // Validate size
        if (pose.size() != 6)
        {
            std::cerr << "❌ MoveToCartesianPose: pose must have 6 elements "
                         "(x,y,z,roll,pitch,yaw)"
                      << std::endl;
            return bt::Status::FAILURE;
        }

        // Convert to Pose type (x, y, z, roll, pitch, yaw)
        Pose target_pose;
        target_pose << pose[0], pose[1], pose[2], pose[3], pose[4], pose[5];

        std::cout << "🎯 Target pose: [" << pose[0] << ", " << pose[1] << ", "
                  << pose[2] << ", " << pose[3] << ", " << pose[4] << ", "
                  << pose[5] << "]" << std::endl;

        // Get end effector from robot (assume first end effector)
        auto const& end_effectors = m_robot.blueprint().endEffectors();
        if (end_effectors.empty())
        {
            std::cerr << "❌ MoveToCartesianPose: no end effector found"
                      << std::endl;
            return bt::Status::FAILURE;
        }

        std::cout << "🔧 Using end effector: " << end_effectors[0].get().name()
                  << std::endl;

        // Store current joint positions
        auto current_positions = m_robot.states().joint_positions;

        std::cout << "📍 Current joint positions: [";
        for (size_t i = 0; i < current_positions.size(); ++i)
        {
            std::cout << current_positions[i];
            if (i < current_positions.size() - 1)
                std::cout << ", ";
        }
        std::cout << "]" << std::endl;

        // Solve IK (modifies robot state)
        if (!m_ik_solver.solve(m_robot, end_effectors[0].get(), target_pose))
        {
            std::cerr << "❌ MoveToCartesianPose: IK solver failed";
            if (m_ik_solver.converged())
            {
                std::cerr << " (converged but outside limits?)";
            }
            else
            {
                std::cerr << " (did not converge after "
                          << m_ik_solver.numIterations()
                          << " iterations, error: " << m_ik_solver.poseError()
                          << ")";
            }
            std::cerr << std::endl;

            // Calculate distance from current end effector to target
            auto current_ee_pose = end_effectors[0].get().worldTransform();
            double dx = target_pose(0) - current_ee_pose(0, 3);
            double dy = target_pose(1) - current_ee_pose(1, 3);
            double dz = target_pose(2) - current_ee_pose(2, 3);
            double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

            std::cerr << "📏 Distance to target: " << distance << " m"
                      << std::endl;
            std::cerr << "📍 Current EE position: [" << current_ee_pose(0, 3)
                      << ", " << current_ee_pose(1, 3) << ", "
                      << current_ee_pose(2, 3) << "]" << std::endl;

            // Restore original positions
            m_robot.setJointPositions(current_positions);
            return bt::Status::FAILURE;
        }

        std::cout << "✅ IK solved successfully" << std::endl;

        // Get the computed joint positions from IK
        auto target_joint_positions = m_robot.states().joint_positions;

        // Restore current positions (trajectory will handle the movement)
        m_robot.setJointPositions(current_positions);

        // Get duration from blackboard or use default
        auto [duration, duration_found] =
            m_blackboard->get<double>("movement_duration");
        if (!duration_found)
        {
            duration = 2.0; // Default 2 seconds
        }

        // Start trajectory
        if (!m_trajectory_controller.goToWaypoint(
                current_positions, target_joint_positions, duration))
        {
            std::cerr << "❌ MoveToCartesianPose: failed to start trajectory"
                      << std::endl;
            return bt::Status::FAILURE;
        }

        m_started = true;
        std::cout << "📍 MoveToCartesianPose action started: [" << pose[0]
                  << ", " << pose[1] << ", " << pose[2] << ", " << pose[3]
                  << ", " << pose[4] << ", " << pose[5] << "]" << std::endl;
        return bt::Status::RUNNING;
    }

    // Subsequent calls: check if trajectory is finished
    if (m_trajectory_controller.isPlaying())
    {
        return bt::Status::RUNNING;
    }

    // Trajectory finished
    m_started = false;
    std::cout << "✅ MoveToCartesianPose action completed" << std::endl;
    return bt::Status::SUCCESS;
}

void MoveToCartesianPoseAction::onHalt()
{
    if (m_started)
    {
        m_trajectory_controller.stop();
        m_started = false;
        std::cout << "⏹️ MoveToCartesianPose action halted" << std::endl;
    }
}

// ============================================================================
// OpenGripperAction
// ============================================================================

OpenGripperAction::OpenGripperAction(bt::Blackboard::Ptr p_blackboard)
    : bt::Leaf(p_blackboard)
{
}

bt::Status OpenGripperAction::onRunning()
{
    std::cout << "🤏 OpenGripper action executed" << std::endl;
    return bt::Status::SUCCESS;
}

// ============================================================================
// CloseGripperAction
// ============================================================================

CloseGripperAction::CloseGripperAction(bt::Blackboard::Ptr p_blackboard)
    : bt::Leaf(p_blackboard)
{
}

bt::Status CloseGripperAction::onRunning()
{
    std::cout << "🤜 CloseGripper action executed" << std::endl;
    return bt::Status::SUCCESS;
}

// ============================================================================
// IsAtPoseCondition
// ============================================================================

IsAtPoseCondition::IsAtPoseCondition(Robot& p_robot,
                                     bt::Blackboard::Ptr p_blackboard)
    : bt::Leaf(p_blackboard), m_robot(p_robot)
{
}

bt::Status IsAtPoseCondition::onRunning()
{
    // Get target pose from blackboard
    auto [target_pose_vec, found] =
        m_blackboard->get<std::vector<double>>("target_pose");
    if (!found || target_pose_vec.size() != 6)
    {
        std::cerr
            << "❌ IsAtPose: target_pose not found or invalid in blackboard"
            << std::endl;
        return bt::Status::FAILURE;
    }

    // Get tolerance from blackboard
    auto [tolerance, tol_found] = m_blackboard->get<double>("tolerance");
    if (!tol_found)
    {
        tolerance = 0.01; // Default tolerance
    }

    // Convert to Pose type
    Pose target_pose;
    target_pose << target_pose_vec[0], target_pose_vec[1], target_pose_vec[2],
        target_pose_vec[3], target_pose_vec[4], target_pose_vec[5];

    // Get current end effector pose
    // TODO: Get actual current pose from robot
    // For now, just return success
    std::cout << "🎯 IsAtPose condition checked (tolerance: " << tolerance
              << ")" << std::endl;

    // Pose current_pose = getCurrentEndEffectorPose(m_robot);
    // Pose error = calculatePoseError(target_pose, current_pose);
    // double position_error = error.head<3>().norm();
    // double orientation_error = error.tail<3>().norm();
    // if (position_error < tolerance && orientation_error < tolerance)
    // {
    //     return bt::Status::SUCCESS;
    // }
    // return bt::Status::FAILURE;

    return bt::Status::SUCCESS;
}

// ============================================================================
// HasTargetCondition
// ============================================================================

HasTargetCondition::HasTargetCondition(bt::Blackboard::Ptr p_blackboard)
    : bt::Leaf(p_blackboard)
{
}

bt::Status HasTargetCondition::onRunning()
{
    // Get target_name from blackboard
    auto [target_name, found] = m_blackboard->get<std::string>("target_name");
    if (!found || target_name.empty())
    {
        std::cout << "⚠️ HasTarget condition: target_name not found"
                  << std::endl;
        return bt::Status::FAILURE;
    }

    std::cout << "✅ HasTarget condition: " << target_name << std::endl;
    return bt::Status::SUCCESS;
}

// ============================================================================
// Registration Helper
// ============================================================================

void registerRobotActions(bt::NodeFactory& p_factory,
                          Robot& p_robot,
                          TeachPendant& p_teach_pendant,
                          IKSolver& p_ik_solver,
                          TrajectoryController& p_trajectory_controller,
                          bt::Blackboard::Ptr p_blackboard)
{
    // Register action nodes
    p_factory.registerNode(
        "Homing",
        [&p_robot,
         &p_trajectory_controller,
         p_blackboard]() -> std::unique_ptr<bt::Node>
        {
            return std::make_unique<HomingAction>(
                p_robot, p_trajectory_controller, p_blackboard);
        });

    p_factory.registerNode(
        "MoveToJointPose",
        [&p_robot,
         &p_trajectory_controller,
         p_blackboard]() -> std::unique_ptr<bt::Node>
        {
            return std::make_unique<MoveToJointPoseAction>(
                p_robot, p_trajectory_controller, p_blackboard);
        });

    p_factory.registerNode(
        "MoveToCartesianPose",
        [&p_robot, &p_ik_solver, &p_trajectory_controller, p_blackboard]()
            -> std::unique_ptr<bt::Node>
        {
            return std::make_unique<MoveToCartesianPoseAction>(
                p_robot, p_ik_solver, p_trajectory_controller, p_blackboard);
        });

    p_factory.registerNode(
        "OpenGripper",
        [p_blackboard]() -> std::unique_ptr<bt::Node>
        { return std::make_unique<OpenGripperAction>(p_blackboard); });

    p_factory.registerNode(
        "CloseGripper",
        [p_blackboard]() -> std::unique_ptr<bt::Node>
        { return std::make_unique<CloseGripperAction>(p_blackboard); });

    // Register condition nodes
    p_factory.registerNode(
        "IsAtPose",
        [&p_robot, p_blackboard]() -> std::unique_ptr<bt::Node>
        { return std::make_unique<IsAtPoseCondition>(p_robot, p_blackboard); });

    p_factory.registerNode(
        "HasTarget",
        [p_blackboard]() -> std::unique_ptr<bt::Node>
        { return std::make_unique<HasTargetCondition>(p_blackboard); });
}

} // namespace robotik
