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
#include "Robotik/Core/Robot/Blueprint/Frame.hpp"

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
      m_states(m_blueprint.numJoints(), m_blueprint.numLinks())
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
    m_blueprint.forEachJoint(
        [this, &p_joint_positions](Joint& joint, size_t index)
        {
            joint.position(p_joint_positions[index]);
            m_states.joint_positions[index] = p_joint_positions[index];
        });
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
            m_states.joint_positions[index] = value;
            m_states.joint_velocities[index] = 0.0;
            m_states.joint_accelerations[index] = 0.0;
        });
}

// ----------------------------------------------------------------------------
bool Robot::setHomePosition(const JointValues& p_home_position)
{
    if (p_home_position.size() != m_blueprint.numJoints())
    {
        std::cerr << "⚠️  Invalid home position size: expected "
                  << m_blueprint.numJoints() << ", got "
                  << p_home_position.size() << std::endl;
        return false;
    }

    if (!areJointPositionsValid(p_home_position))
    {
        std::cerr
            << "⚠️  Home position contains out-of-limits values. Clamping..."
            << std::endl;
        m_home_position = clampJointPositions(p_home_position);
        return false; // Indicate modification
    }

    m_home_position = p_home_position;
    return true;
}

// ----------------------------------------------------------------------------
void Robot::setHomePosition()
{
    if (!m_home_position.empty() &&
        m_home_position.size() == m_blueprint.numJoints())
    {
        setJointPositions(m_home_position);
    }
}

// ----------------------------------------------------------------------------
bool Robot::areJointPositionsValid(const JointValues& p_positions) const
{
    if (p_positions.size() != m_blueprint.numJoints())
        return false;

    bool valid = true;
    m_blueprint.forEachJoint(
        [&p_positions, &valid](Joint const& joint, size_t index)
        {
            auto [min, max] = joint.limits();
            if (p_positions[index] < min || p_positions[index] > max)
            {
                valid = false;
            }
        });

    return valid;
}

// ----------------------------------------------------------------------------
JointValues Robot::clampJointPositions(const JointValues& p_positions) const
{
    JointValues clamped = p_positions;

    if (clamped.size() != m_blueprint.numJoints())
    {
        std::cerr << "⚠️  Cannot clamp: invalid size" << std::endl;
        return clamped;
    }

    m_blueprint.forEachJoint(
        [&clamped](Joint const& joint, size_t index)
        {
            auto [min, max] = joint.limits();
            if (clamped[index] < min)
            {
                std::cerr << "⚠️  Joint " << index << " (" << joint.name()
                          << ") clamped from " << clamped[index] << " to "
                          << min << std::endl;
                clamped[index] = min;
            }
            else if (clamped[index] > max)
            {
                std::cerr << "⚠️  Joint " << index << " (" << joint.name()
                          << ") clamped from " << clamped[index] << " to "
                          << max << std::endl;
                clamped[index] = max;
            }
        });

    return clamped;
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
            m_states.joint_positions[index] = new_pos;
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

// ----------------------------------------------------------------------------
void Robot::traverse(ConstNodeVisitor& visitor) const
{
    if (m_blueprint.hasRoot() && m_blueprint.enabled())
    {
        m_blueprint.root().traverse(visitor);
    }
}

// ----------------------------------------------------------------------------
Frame& Robot::addFrame(std::string const& p_name, Transform const& p_transform)
{
    return addFrameToNode(m_blueprint.root(), p_name, p_transform);
}

// ----------------------------------------------------------------------------
Frame& Robot::addFrameToNode(Node& p_parent,
                             std::string const& p_name,
                             Transform const& p_transform)
{
    Frame& frame = p_parent.createChild<Frame>(p_name);
    frame.localTransform(p_transform);
    return frame;
}

// ----------------------------------------------------------------------------
void Robot::attachToFrame(Frame& p_frame)
{
    if (!m_blueprint.hasRoot())
    {
        return;
    }

    // Create a new root frames with the same properties as the provided one
    Node::Ptr new_root = Node::create<Frame>(p_frame.name());
    new_root->localTransform(p_frame.localTransform());

    // Get reference to current root
    Node& old_root = m_blueprint.root();

    // Move all children from old root to new root
    auto& old_children = old_root.children();
    for (auto& child : old_children)
    {
        new_root->addChild(std::move(child));
    }
    old_children.clear();

    // Add the old root as a child of the new frames root
    // Note: We can't move the old root since it's owned by Blueprint,
    // so we create a wrapper or work with references
    // For now, the old root structure is preserved as children of new root
    // The new root becomes the blueprint root
    m_blueprint.root(std::move(new_root));
}

} // namespace robotik
