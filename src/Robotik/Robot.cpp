/**
 * @file Robot.cpp
 * @brief Implementation of Robot class (kinematic computations).
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#include "Robotik/Core/Robot.hpp"
#include "Robotik/Core/Conversions.hpp"
// #include "Robotik/Core/IKSolver.hpp"

namespace robotik
{

// ----------------------------------------------------------------------------
Robot::Robot(std::string const& p_name, Node::Ptr p_root)
    : Robot(p_name, Hierarchy(std::move(p_root)))
{
}

// ----------------------------------------------------------------------------
Robot::Robot(std::string const& p_name, Hierarchy&& p_hierarchy)
    : m_name(p_name),
      m_hierarchy(std::move(p_hierarchy)),
      m_state(m_hierarchy.numJoints(), m_hierarchy.numLinks())
{
    // Modify the mesh path to be relative to the robot name
    m_hierarchy.forEachLink(
        [this](Link& p_link)
        {
            auto& geometry = p_link.geometry();
            geometry.meshPath(m_name + "/" + geometry.meshPath());
        });
}

// ----------------------------------------------------------------------------
void Robot::forwardKinematics(State& p_state)
{
    m_hierarchy.forEachJoint(
        [&](Joint& p_joint, size_t p_index)
        {
            // The joint position() method automatically updates world
            // transforms via the hierarchy tree traversal
            p_joint.position(p_state.joint_positions[p_index]);
        });
}

// ----------------------------------------------------------------------------
Pose Robot::calculatePoseError(Pose const& p_target_pose,
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
void Robot::setNeutralPosition()
{
    m_hierarchy.forEachJoint(
        [this](Joint& p_joint, size_t p_index)
        {
            auto [min, max] = p_joint.limits();
            double value = (min + max) / 2.0;
            p_joint.position(value);
            m_state.joint_positions[p_index] = value;
            m_state.joint_velocities[p_index] = 0.0;
            m_state.joint_accelerations[p_index] = 0.0;
        });
}

// ----------------------------------------------------------------------------
Jacobian const& Robot::computeJacobian(State& p_state,
                                       Node const& p_end_effector)
{
    // Resize the Jacobian matrix to the number of joints
    Jacobian& J = p_state.jacobian;
    J.resize(6, m_hierarchy.numJoints());

    // Position of the end effector
    Transform const& end_effector_transform = p_end_effector.worldTransform();
    Eigen::Vector3d end_pos = end_effector_transform.block<3, 1>(0, 3);

    m_hierarchy.forEachJoint(
        [&J, &end_pos](Joint const& joint, size_t index)
        {
            // Transformation of the joint in the global space
            Transform const& joint_transform = joint.worldTransform();

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
                J.block<3, 1>(0, index) = v;
                J.block<3, 1>(3, index) = joint_axis;
            }
            else if (joint.type() == Joint::Type::PRISMATIC)
            {
                // Contribution to linear velocity: axis
                J.block<3, 1>(0, index) = joint_axis;
                J.block<3, 1>(3, index) = Eigen::Vector3d::Zero();
            }
        });

    return J;
}

// ----------------------------------------------------------------------------
bool Robot::inverseKinematics(State& p_state,
                              Node const& p_end_effector,
                              Pose const& p_target_pose,
                              IKSolver& p_solver)
{
    // The IK solver will work with the old Robot interface temporarily
    // This is a bridge until we update IKSolver
    // For now, just return false as we need to update IKSolver first
    return false;
}

} // namespace robotik
