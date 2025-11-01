/**
 * @file Robot.cpp
 * @brief Implementation of Robot class (kinematic computations).
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#include "Robotik/Core/Robot/Robot.hpp"
#include "Robotik/Core/Robot/Blueprint/Blueprint.hpp"

namespace robotik
{

// ----------------------------------------------------------------------------
Transform Robot::computeJointTransform(Joint::Type p_type,
                                       Eigen::Vector3d const& p_axis,
                                       double p_position)
{
    Transform T = Transform::Identity();

    switch (p_type)
    {
        case Joint::Type::REVOLUTE:
        case Joint::Type::CONTINUOUS:
        {
            // Rotation around axis
            Eigen::AngleAxisd rotation(p_position, p_axis);
            T.block<3, 3>(0, 0) = rotation.toRotationMatrix();
            break;
        }
        case Joint::Type::PRISMATIC:
        {
            // Translation along axis
            T.block<3, 1>(0, 3) = p_axis * p_position;
            break;
        }
        case Joint::Type::FIXED:
        default:
        {
            // Identity (no motion)
            break;
        }
    }

    return T;
}

// ----------------------------------------------------------------------------
Robot::Robot(std::string const& p_name, Blueprint&& p_blueprint)
    : m_name(p_name),
      m_blueprint(std::move(p_blueprint)),
      m_state(m_blueprint.numJoints(), m_blueprint.numLinks())
{
}

// ----------------------------------------------------------------------------
void Robot::setJointPositions(State& p_state)
{
    // Cache-friendly forward kinematics using flat arrays
    // This implementation does NOT modify the Blueprint (immutable model)
    // All transforms are computed and stored in State only

    // Step 1: Compute joint local transforms from positions
    m_blueprint.forEachJointData(
        [&](JointData const& joint, size_t i)
        {
            p_state.joint_local_transforms[i] = computeJointTransform(
                joint.type, joint.axis, p_state.joint_positions[i]);
        });

    // Step 2: Initialize ALL link transforms to identity first
    m_blueprint.forEachLinkData(
        [&](LinkData const& link, size_t i)
        {
            if (link.parent_joint_index == SIZE_MAX)
            {
                // Root link (no parent joint) - at world origin
                p_state.link_transforms[i] = link.placement; // Usually Identity
            }
            else
            {
                // Initialize to identity, will be updated in the passes below
                p_state.link_transforms[i] = Transform::Identity();
            }
        });

    // Step 3: Propagate transforms through kinematic chain
    // The buildFlatArraysRecursive creates arrays in depth-first order:
    // Link0 -> Joint0 -> Link1 -> Joint1 -> Link2 -> ...
    // So we can compute in a single pass by processing in order

    // Track which transforms have been computed
    std::vector<bool> joint_computed(p_state.joint_transforms.size(), false);
    std::vector<bool> link_computed(p_state.link_transforms.size(), false);

    // Mark root links as computed
    m_blueprint.forEachLinkData(
        [&](LinkData const& link, size_t i)
        {
            if (link.parent_joint_index == SIZE_MAX)
            {
                link_computed[i] = true;
            }
        });

    // Compute transforms iteratively until all are done
    bool changed = true;
    int max_iterations = 100; // Safety limit
    int iteration = 0;

    while (changed && iteration < max_iterations)
    {
        changed = false;
        iteration++;

        // Try to compute joint transforms
        m_blueprint.forEachJointData(
            [&](JointData const& joint, size_t i)
            {
                if (joint_computed[i])
                    return; // Already computed

                // Check if parent link is computed
                if (joint.parent_link_index == SIZE_MAX ||
                    (joint.parent_link_index < link_computed.size() &&
                     link_computed[joint.parent_link_index]))
                {
                    Transform parent_link_transform;

                    if (joint.parent_link_index == SIZE_MAX)
                    {
                        parent_link_transform = Transform::Identity();
                    }
                    else
                    {
                        parent_link_transform =
                            p_state.link_transforms[joint.parent_link_index];
                    }

                    // Compute world transform
                    p_state.joint_transforms[i] =
                        parent_link_transform * joint.placement *
                        p_state.joint_local_transforms[i];

                    joint_computed[i] = true;
                    changed = true;
                }
            });

        // Try to compute link transforms
        m_blueprint.forEachLinkData(
            [&](LinkData const& link, size_t i)
            {
                if (link_computed[i])
                    return; // Already computed

                // Check if parent joint is computed
                if (link.parent_joint_index < joint_computed.size() &&
                    joint_computed[link.parent_joint_index])
                {
                    Transform parent_transform =
                        p_state.joint_transforms[link.parent_joint_index];
                    p_state.link_transforms[i] =
                        parent_transform * link.placement;

                    link_computed[i] = true;
                    changed = true;
                }
            });
    }
}

// ----------------------------------------------------------------------------
void Robot::setJointPositions(const JointValues& p_joint_positions)
{
    // Copy positions to internal state
    m_state.joint_positions = p_joint_positions;
    // Compute forward kinematics
    setJointPositions(m_state);
}

// ----------------------------------------------------------------------------
void Robot::setNeutralPosition()
{
    // Set all joints to neutral position (mid-range) using flat array data
    m_blueprint.forEachJointData(
        [this](JointData const& joint, size_t i)
        {
            double value = (joint.position_min + joint.position_max) / 2.0;
            m_state.joint_positions[i] = value;
            m_state.joint_velocities[i] = 0.0;
            m_state.joint_accelerations[i] = 0.0;
        });

    // Compute forward kinematics
    setJointPositions(m_state);
}

// ----------------------------------------------------------------------------
void Robot::applyJointTargetsWithSpeedLimit(const JointValues& q_target,
                                            double dt)
{
    // Apply velocity-limited motion using flat array data
    m_blueprint.forEachJointData(
        [&](JointData const& joint, size_t i)
        {
            double current_pos = m_state.joint_positions[i];
            double diff = q_target[i] - current_pos;

            // Get maximum velocity for this joint
            double max_speed = joint.velocity_max;

            // If no velocity limit set, use a reasonable default
            if (max_speed <= 0.0)
            {
                max_speed = 1.0; // Default 1 rad/s or 1 m/s
            }

            // Clamp the step to respect velocity limits
            double max_step = max_speed * dt;
            double step = std::clamp(diff, -max_step, max_step);

            // Apply the clamped position
            m_state.joint_positions[i] = current_pos + step;
        });

    // Compute forward kinematics
    setJointPositions(m_state);
}

// ----------------------------------------------------------------------------
Jacobian const& Robot::computeJacobian(State& p_state,
                                       Node const& p_end_effector)
{
    // Cache-friendly Jacobian using flat arrays
    // Resize the Jacobian matrix to the number of joints
    Jacobian& J = p_state.jacobian;
    J.resize(6, m_blueprint.numJoints());

    // Position of the end effector (from tree node for backward compatibility)
    Transform const& end_effector_transform = p_end_effector.worldTransform();
    Eigen::Vector3d end_pos = end_effector_transform.block<3, 1>(0, 3);

    // Iterate over joints using flat array data
    m_blueprint.forEachJointData(
        [&](JointData const& joint, size_t i)
        {
            // Get joint transform from State (computed by forward kinematics)
            Transform const& joint_transform = p_state.joint_transforms[i];

            // Position of the joint
            Eigen::Vector3d joint_pos = joint_transform.block<3, 1>(0, 3);

            // Orientation of the joint axis in the global space
            Eigen::Vector3d joint_axis =
                joint_transform.block<3, 3>(0, 0) * joint.axis;

            if ((joint.type == Joint::Type::REVOLUTE) ||
                (joint.type == Joint::Type::CONTINUOUS))
            {
                // Contribution to linear velocity: cross(axis, (end - joint))
                Eigen::Vector3d r = end_pos - joint_pos;
                Eigen::Vector3d v = joint_axis.cross(r);

                // Fill the Jacobian
                J.block<3, 1>(0, i) = v;
                J.block<3, 1>(3, i) = joint_axis;
            }
            else if (joint.type == Joint::Type::PRISMATIC)
            {
                // Contribution to linear velocity: axis
                J.block<3, 1>(0, i) = joint_axis;
                J.block<3, 1>(3, i) = Eigen::Vector3d::Zero();
            }
            else
            {
                // FIXED joint contributes nothing
                J.block<3, 1>(0, i) = Eigen::Vector3d::Zero();
                J.block<3, 1>(3, i) = Eigen::Vector3d::Zero();
            }
        });

    return J;
}

} // namespace robotik
