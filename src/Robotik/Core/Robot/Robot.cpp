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
Robot::Robot(std::string const& p_name, Node::Ptr p_root)
    : Robot(p_name, Blueprint(std::move(p_root)))
{
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
    m_blueprint.forEachJoint(
        [&p_state](Joint& joint, size_t index)
        {
            // The joint position() method automatically updates world
            // transforms via the blueprint tree traversal
            joint.position(p_state.joint_positions[index]);
        });
}

// ----------------------------------------------------------------------------
void Robot::setJointPositions(const JointValues& p_joint_positions)
{
    m_blueprint.forEachJoint([&p_joint_positions](Joint& joint, size_t index)
                             { joint.position(p_joint_positions[index]); });
}

// ----------------------------------------------------------------------------
void Robot::setNeutralPosition()
{
    m_blueprint.forEachJoint(
        [this](Joint& joint, size_t index)
        {
            auto [min, max] = joint.limits();
            double value = (min + max) / 2.0;
            joint.position(value);
            m_state.joint_positions[index] = value;
            m_state.joint_velocities[index] = 0.0;
            m_state.joint_accelerations[index] = 0.0;
        });
}

// ----------------------------------------------------------------------------
void Robot::applyJointTargetsWithSpeedLimit(const JointValues& q_target,
                                            double dt)
{
    m_blueprint.forEachJoint(
        [&q_target, dt, this](Joint& joint, size_t index)
        {
            double current_pos = joint.position();
            double diff = q_target[index] - current_pos;

            // Get maximum velocity for this joint
            double max_speed = joint.maxVelocity();

            // If no velocity limit set, use a reasonable default
            if (max_speed <= 0.0)
            {
                max_speed = 1.0; // Default 1 rad/s or 1 m/s
            }

            // Clamp the step to respect velocity limits
            double max_step = max_speed * dt;
            double step = std::clamp(diff, -max_step, max_step);

            // Apply the clamped position
            double new_pos = current_pos + step;
            joint.position(new_pos);

            // Update state
            m_state.joint_positions[index] = new_pos;
        });
}

// ----------------------------------------------------------------------------
Jacobian const& Robot::computeJacobian(State& p_state,
                                       Node const& p_end_effector)
{
    // Resize the Jacobian matrix to the number of joints
    Jacobian& J = p_state.jacobian;
    J.resize(6, m_blueprint.numJoints());

    // Position of the end effector
    Transform const& end_effector_transform = p_end_effector.worldTransform();
    Eigen::Vector3d end_pos = end_effector_transform.block<3, 1>(0, 3);

    m_blueprint.forEachJoint(
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

} // namespace robotik
