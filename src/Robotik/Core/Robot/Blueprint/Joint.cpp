/**
 * @file Joint.cpp
 * @brief Joint class - Representation of a robotic joint.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#include "Robotik/Core/Robot/Blueprint/Joint.hpp"
#include "Robotik/Core/Robot/Blueprint/NodeVisitor.hpp"

namespace robotik
{

// ----------------------------------------------------------------------------
Joint::Joint(std::string const& p_name,
             Joint::Type p_type,
             const Eigen::Vector3d& p_axis,
             size_t p_index)
    : Node(p_name),
      m_type(p_type),
      m_index(p_index),
      m_axis(p_axis.normalized())
{
    switch (p_type)
    {
        case Joint::Type::CONTINUOUS:
        case Joint::Type::REVOLUTE:
            limits(-M_PI, M_PI);
            position(0.0);
            break;
        case Joint::Type::PRISMATIC:
            limits(-1.0, 1.0);
            position(0.0);
            break;
        case Joint::Type::FIXED:
            m_position_min = m_position_max = 0.0;
            m_position = 0.0;
            m_joint_transform = Eigen::Matrix4d::Identity();
            m_combined_local_transform = Eigen::Matrix4d::Identity();
            break;
    }
}

// ----------------------------------------------------------------------------
void Joint::position(double p_velocity, double p_dt)
{
    position(m_position + p_dt * p_velocity);
}

// ----------------------------------------------------------------------------
// TODO: si continuous wrap to -M_PI to M_PI
void Joint::position(double p_position)
{
    if (m_type == Joint::Type::FIXED)
        return;

    if (m_type != Joint::Type::CONTINUOUS)
    {
        if (p_position < m_position_min)
            p_position = m_position_min;
        if (p_position > m_position_max)
            p_position = m_position_max;
    }

    // Compute the joint's local transformation matrix
    Eigen::Matrix4d joint_transform = Eigen::Matrix4d::Identity();
    if (m_type == Joint::Type::PRISMATIC)
    {
        // Translation along the axis
        joint_transform.block<3, 1>(0, 3) = m_axis * p_position;
    }
    else if (m_type == Joint::Type::CONTINUOUS ||
             m_type == Joint::Type::REVOLUTE)
    {
        // Create the rotation matrix around the axis
        Eigen::AngleAxisd joint_rotation(p_position, m_axis);
        joint_transform.block<3, 3>(0, 0) = joint_rotation.toRotationMatrix();
    }

    // Update the joint's position and transformation
    m_position = p_position;
    m_joint_transform = joint_transform;
    m_combined_local_transform = m_local_transform * m_joint_transform;

    // Update the world transform
    updateWorldTransforms(); // FIXME find a lazy way to do this
}

// ----------------------------------------------------------------------------
void Joint::velocity(double p_acceleration, double p_dt)
{
    double v = m_velocity + p_dt * p_acceleration;
    m_velocity = std::max(-maxVelocity(), std::min(maxVelocity(), v));
}

// ----------------------------------------------------------------------------
void Joint::localTransform(Transform const& p_transform)
{
    m_local_transform = p_transform;
    m_combined_local_transform = m_local_transform * m_joint_transform;
    updateWorldTransforms(); // FIXME find a lazy way to do this
}

// ----------------------------------------------------------------------------
Transform const& Joint::localTransform() const
{
    return m_combined_local_transform;
}

// ----------------------------------------------------------------------------
void Joint::accept(NodeVisitor& visitor)
{
    visitor.visit(*this);
}

// ----------------------------------------------------------------------------
void Joint::accept(ConstNodeVisitor& visitor) const
{
    visitor.visit(*this);
}

// ----------------------------------------------------------------------------
void Joint::setIndex(size_t p_index)
{
    m_index = p_index;
}

} // namespace robotik