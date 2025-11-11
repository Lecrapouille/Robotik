/**
 * @file RobotActions.hpp
 * @brief Built-in robot action and condition nodes for behavior trees.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#pragma once

#include "Robotik/Core/BehaviorTree/BehaviorTree.hpp"

namespace robotik
{

// Forward declarations
class Robot;
class TeachPendant;
class IKSolver;
class TrajectoryController;

// ****************************************************************************
//! \brief Action node that moves the robot to home position.
// ****************************************************************************
class HomingAction final: public bt::Leaf
{
public:

    HomingAction(Robot& p_robot,
                 TrajectoryController& p_trajectory_controller,
                 bt::Blackboard::Ptr p_blackboard);
    bt::Status onRunning() override;
    void onHalt() override;
    // Visitor pattern required by bt::Node but not used for custom actions
    void accept(bt::ConstBehaviorTreeVisitor& /*p_visitor*/) const override {}
    void accept(bt::BehaviorTreeVisitor& /*p_visitor*/) override {}

private:

    Robot& m_robot;
    TrajectoryController& m_trajectory_controller;
    bool m_started = false;
};

// ****************************************************************************
//! \brief Action node that moves the robot to a joint pose.
//! Reads 'joint_positions' parameter from blackboard.
// ****************************************************************************
class MoveToJointPoseAction final: public bt::Leaf
{
public:

    MoveToJointPoseAction(Robot& p_robot,
                          TrajectoryController& p_trajectory_controller,
                          bt::Blackboard::Ptr p_blackboard);
    bt::Status onRunning() override;
    void onHalt() override;
    // Visitor pattern required by bt::Node but not used for custom actions
    void accept(bt::ConstBehaviorTreeVisitor& /*p_visitor*/) const override {}
    void accept(bt::BehaviorTreeVisitor& /*p_visitor*/) override {}

private:

    Robot& m_robot;
    TrajectoryController& m_trajectory_controller;
    bool m_started = false;
};

// ****************************************************************************
//! \brief Action node that moves the robot to a cartesian pose.
//! Reads 'pose' parameter from blackboard.
// ****************************************************************************
class MoveToCartesianPoseAction final: public bt::Leaf
{
public:

    MoveToCartesianPoseAction(Robot& p_robot,
                              IKSolver& p_ik_solver,
                              TrajectoryController& p_trajectory_controller,
                              bt::Blackboard::Ptr p_blackboard);
    bt::Status onRunning() override;
    void onHalt() override;
    // Visitor pattern required by bt::Node but not used for custom actions
    void accept(bt::ConstBehaviorTreeVisitor& /*p_visitor*/) const override {}
    void accept(bt::BehaviorTreeVisitor& /*p_visitor*/) override {}

private:

    Robot& m_robot;
    IKSolver& m_ik_solver;
    TrajectoryController& m_trajectory_controller;
    bool m_started = false;
};

// ****************************************************************************
//! \brief Action node that opens the gripper.
// ****************************************************************************
class OpenGripperAction final: public bt::Leaf
{
public:

    explicit OpenGripperAction(bt::Blackboard::Ptr p_blackboard);
    bt::Status onRunning() override;
    // Visitor pattern required by bt::Node but not used for custom actions
    void accept(bt::ConstBehaviorTreeVisitor& /*p_visitor*/) const override {}
    void accept(bt::BehaviorTreeVisitor& /*p_visitor*/) override {}
};

// ****************************************************************************
//! \brief Action node that closes the gripper.
// ****************************************************************************
class CloseGripperAction final: public bt::Leaf
{
public:

    explicit CloseGripperAction(bt::Blackboard::Ptr p_blackboard);
    bt::Status onRunning() override;
    // Visitor pattern required by bt::Node but not used for custom actions
    void accept(bt::ConstBehaviorTreeVisitor& /*p_visitor*/) const override {}
    void accept(bt::BehaviorTreeVisitor& /*p_visitor*/) override {}
};

// ****************************************************************************
//! \brief Condition node that checks if robot is at target pose.
//! Uses calculatePoseError to check if within tolerance.
//! Reads 'target_pose' and 'tolerance' from blackboard.
// ****************************************************************************
class IsAtPoseCondition final: public bt::Leaf
{
public:

    IsAtPoseCondition(Robot& p_robot, bt::Blackboard::Ptr p_blackboard);
    bt::Status onRunning() override;
    // Visitor pattern required by bt::Node but not used for custom conditions
    void accept(bt::ConstBehaviorTreeVisitor& /*p_visitor*/) const override {}
    void accept(bt::BehaviorTreeVisitor& /*p_visitor*/) override {}

private:

    Robot& m_robot;
};

// ****************************************************************************
//! \brief Condition node that checks if a target exists in the blackboard.
//! Reads 'target_name' from blackboard and checks if it exists.
// ****************************************************************************
class HasTargetCondition final: public bt::Leaf
{
public:

    explicit HasTargetCondition(bt::Blackboard::Ptr p_blackboard);
    bt::Status onRunning() override;
    // Visitor pattern required by bt::Node but not used for custom conditions
    void accept(bt::ConstBehaviorTreeVisitor& /*p_visitor*/) const override {}
    void accept(bt::BehaviorTreeVisitor& /*p_visitor*/) override {}
};

// ****************************************************************************
//! \brief Helper function to register all built-in robot actions.
//! \param p_factory Node factory to register actions in.
//! \param p_robot Robot to control.
//! \param p_teach_pendant Teach pendant for robot control.
//! \param p_ik_solver IK solver for cartesian movements.
//! \param p_trajectory_controller Trajectory controller for smooth movements.
//! \param p_blackboard Blackboard for sharing data.
// ****************************************************************************
void registerRobotActions(bt::NodeFactory& p_factory,
                          Robot& p_robot,
                          TeachPendant& p_teach_pendant,
                          IKSolver& p_ik_solver,
                          TrajectoryController& p_trajectory_controller,
                          bt::Blackboard::Ptr p_blackboard);

} // namespace robotik
