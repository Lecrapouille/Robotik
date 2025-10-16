/**
 * @file Robot.cpp
 * @brief Implementation of Robot class (kinematic computations).
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#include "Robotik/Core/Robot_new.hpp"
#include "Robotik/Core/Conversions.hpp"
#include "Robotik/Core/IKSolver.hpp"

namespace robotik
{

// ----------------------------------------------------------------------------
Robot_new::Robot_new(Hierarchy const& p_hierarchy) : m_hierarchy(p_hierarchy) {}

// ----------------------------------------------------------------------------
State Robot_new::createState() const
{
    return State(m_hierarchy.numJoints(), m_hierarchy.numLinks());
}

// ----------------------------------------------------------------------------
void Robot_new::forwardKinematics(State& p_state)
{
    // Set joint positions in the hierarchy
    auto& joints = const_cast<Hierarchy&>(m_hierarchy).joints();

    for (size_t i = 0; i < joints.size(); ++i)
    {
        joints[i].get().position(p_state.jointPositions()[i]);
    }

    // The joint position() method automatically updates world transforms
    // via the hierarchy tree traversal

    // Mark Jacobian as dirty since positions changed
    p_state.markJacobianDirty();
}

// ----------------------------------------------------------------------------
Pose Robot_new::calculatePoseError(Pose const& p_target_pose,
                                   Pose const& p_current_pose) const
{
    Pose error;

    // Position error is straightforward
    error.head<3>() = p_target_pose.head<3>() - p_current_pose.head<3>();

    // Orientation error: convert to rotation matrices for proper subtraction
    Eigen::Vector3d target_euler = p_target_pose.tail<3>();
    Eigen::Vector3d current_euler = p_current_pose.tail<3>();

    Eigen::Matrix3d target_rot = utils::eulerToRotation(
        target_euler(0), target_euler(1), target_euler(2));
    Eigen::Matrix3d current_rot = utils::eulerToRotation(
        current_euler(0), current_euler(1), current_euler(2));

    // Compute rotation error: R_error = R_target * R_current^T
    Eigen::Matrix3d error_rot = target_rot * current_rot.transpose();

    // Convert back to Euler angles
    Eigen::Vector3d error_euler = utils::rotationToEuler(error_rot);
    error.tail<3>() = error_euler;

    return error;
}

// ----------------------------------------------------------------------------
Jacobian const&
Robot_new::computeJacobian(State& p_state,
                           hierarchy::Node const& p_end_effector)
{
    // Check if Jacobian is already up to date
    if (!p_state.isJacobianDirty())
    {
        return p_state.jacobian();
    }

    const size_t num_joints = m_hierarchy.numJoints();
    Jacobian& J = p_state.jacobian();
    J.resize(6, num_joints);

    // Position of the end effector
    Transform end_effector_transform = p_end_effector.worldTransform();
    Eigen::Vector3d end_pos = end_effector_transform.block<3, 1>(0, 3);

    auto const& joints = m_hierarchy.joints();
    for (size_t i = 0; i < num_joints; ++i)
    {
        Joint const& joint = joints[i].get();

        // Transformation of the joint in the global space
        Transform joint_transform = joint.worldTransform();

        // Position of the joint
        Eigen::Vector3d joint_pos = joint_transform.block<3, 1>(0, 3);

        // Orientation of the joint axis in the global space
        Eigen::Vector3d joint_axis =
            joint_transform.block<3, 3>(0, 0) * joint.axis();

        if ((joint.type() == Joint::Type::REVOLUTE) ||
            (joint.type() == Joint::Type::CONTINUOUS))
        {
            // Contribution to linear velocity: cross(axis, (end - joint))
            Eigen::Vector3d r = end_pos - joint_pos;
            Eigen::Vector3d v = joint_axis.cross(r);

            // Fill the Jacobian
            J.block<3, 1>(0, i) = v;
            J.block<3, 1>(3, i) = joint_axis;
        }
        else if (joint.type() == Joint::Type::PRISMATIC)
        {
            // Contribution to linear velocity: axis
            J.block<3, 1>(0, i) = joint_axis;
            J.block<3, 1>(3, i) = Eigen::Vector3d::Zero();
        }
    }

    p_state.markJacobianClean();
    return J;
}

// ----------------------------------------------------------------------------
bool Robot_new::inverseKinematics(State& p_state,
                                  hierarchy::Node const& p_end_effector,
                                  Pose const& p_target_pose,
                                  IKSolver& p_solver)
{
    // The IK solver will work with the old Robot interface temporarily
    // This is a bridge until we update IKSolver
    // For now, just return false as we need to update IKSolver first
    return false;
}

} // namespace robotik
