/**
 * @file TeachPendant.cpp
 * @brief Implementation of the teach pendant for interactive control of the
 * robot.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#include "Robotik/Core/Robot/TeachPendant.hpp"
#include "Robotik/Core/Common/Conversions.hpp"
#include "Robotik/Core/Robot/Blueprint/Joint.hpp"

namespace robotik
{

// ----------------------------------------------------------------------------
void TeachPendant::setIKSolver(IKSolver* p_solver)
{
    m_ik_solver = p_solver;
}

// ----------------------------------------------------------------------------
bool TeachPendant::moveJoint(Robot& p_robot,
                             size_t p_joint_idx,
                             double p_delta,
                             double p_speed)
{
    // Check that the index is valid
    if (p_joint_idx >= p_robot.blueprint().numJoints())
    {
        m_error = "Invalid joint index";
        return false;
    }

    // Calculate the new target position
    auto target = p_robot.states().joint_positions;
    target[p_joint_idx] += p_delta * p_speed;

    // Check the limits via forEachJoint
    bool within_limits = true;
    p_robot.blueprint().forEachJoint(
        [&target, &within_limits, p_joint_idx](Joint const& joint, size_t index)
        {
            if (index == p_joint_idx)
            {
                auto [min, max] = joint.limits();
                if (target[index] < min || target[index] > max)
                {
                    within_limits = false;
                }
            }
        });

    if (!within_limits)
    {
        m_error = "Target position is out of limits";
        return false;
    }

    // Apply the movement
    p_robot.setJointPositions(target);

    return true;
}

// ----------------------------------------------------------------------------
bool TeachPendant::moveJoints(Robot& p_robot,
                              const std::vector<double>& p_deltas,
                              double p_speed)
{
    // Check that the number of deltas corresponds to the number of joints
    if (p_deltas.size() != p_robot.blueprint().numJoints())
    {
        m_error =
            "Number of deltas does not correspond to the number of joints";
        return false;
    }

    // Calculate the new target positions
    auto target = p_robot.states().joint_positions;
    for (size_t i = 0; i < p_deltas.size(); ++i)
    {
        target[i] += p_deltas[i] * p_speed;
    }

    // Check all limits
    bool within_limits = true;
    p_robot.blueprint().forEachJoint(
        [&target, &within_limits](Joint const& joint, size_t index)
        {
            auto [min, max] = joint.limits();
            if (target[index] < min || target[index] > max)
            {
                within_limits = false;
            }
        });

    if (!within_limits)
    {
        m_error = "Target position is out of limits";
        return false;
    }

    // Apply the movement
    p_robot.setJointPositions(target);

    return true;
}

// ----------------------------------------------------------------------------
bool TeachPendant::moveCartesian(Robot& p_robot,
                                 Node const* p_end_effector,
                                 const Eigen::Vector3d& p_translation,
                                 double p_speed,
                                 Node const* p_frame)
{
    if (!p_end_effector || !m_ik_solver)
    {
        m_error = "End effector or IK solver not set";
        return false;
    }

    // Get the current transform of the end effector
    Transform current_transform = p_end_effector->worldTransform();

    // Transform translation vector from frame to world coordinates
    Eigen::Vector3d translation_world = p_translation;
    if (p_frame != nullptr)
    {
        // Get rotation matrix from frame to world
        Eigen::Matrix3d R_frame = p_frame->worldTransform().block<3, 3>(0, 0);
        // Transform direction vector from frame to world
        translation_world = R_frame * p_translation;
    }

    // Apply the translation
    current_transform.block<3, 1>(0, 3) += translation_world * p_speed;

    // Convert to 6D pose for the IK solver
    Pose target_pose;
    target_pose.head<3>() = current_transform.block<3, 1>(0, 3);

    // Extract the Euler angles from the rotation
    Eigen::Matrix3d R = current_transform.block<3, 3>(0, 0);
    target_pose.tail<3>() = robotik::rotationToEuler(R);

    // Solve the inverse kinematics
    if (!m_ik_solver->solve(p_robot, *p_end_effector, target_pose))
    {
        m_error = "Failed to solve the inverse kinematics";
        return false;
    }

    // Apply the IK solution
    p_robot.setJointPositions(m_ik_solver->solution());

    return true;
}

// ----------------------------------------------------------------------------
bool TeachPendant::rotateCartesian(Robot& p_robot,
                                   Node const* p_end_effector,
                                   const Eigen::Vector3d& p_rotation_axis,
                                   double p_angle,
                                   double p_speed,
                                   Node const* p_frame)
{
    if (!p_end_effector || !m_ik_solver)
    {
        m_error = "End effector or IK solver not set";
        return false;
    }

    // Get the current transform of the end effector
    Transform current_transform = p_end_effector->worldTransform();

    // Transform rotation axis from frame to world coordinates
    Eigen::Vector3d axis_world = p_rotation_axis.normalized();
    if (p_frame != nullptr)
    {
        // Get rotation matrix from frame to world
        Eigen::Matrix3d R_frame = p_frame->worldTransform().block<3, 3>(0, 0);
        // Transform axis from frame to world
        axis_world = R_frame * p_rotation_axis.normalized();
    }

    // Apply the rotation with speed factor
    double angle_applied = p_angle * p_speed;
    Eigen::AngleAxisd rotation(angle_applied, axis_world);
    Eigen::Matrix3d R_current = current_transform.block<3, 3>(0, 0);
    Eigen::Matrix3d R_new = rotation.toRotationMatrix() * R_current;

    // Update the transform
    current_transform.block<3, 3>(0, 0) = R_new;

    // Convert to 6D pose for the IK solver
    Pose target_pose;
    target_pose.head<3>() = current_transform.block<3, 1>(0, 3);
    target_pose.tail<3>() = robotik::rotationToEuler(R_new);

    // Solve the inverse kinematics
    if (!m_ik_solver->solve(p_robot, *p_end_effector, target_pose))
    {
        m_error = "Failed to solve the inverse kinematics";
        return false;
    }

    // Apply the IK solution
    p_robot.setJointPositions(m_ik_solver->solution());

    return true;
}

} // namespace robotik
