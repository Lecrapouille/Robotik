/**
 * @file Joint.cpp
 * @brief Joint class - Representation of a robotic joint.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#include "Robotik/Core/Robot/Blueprint/Joint.hpp"

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
            break;
    }
}

// ----------------------------------------------------------------------------
void Joint::position(double p_velocity, double p_dt)
{
    position(m_position + p_dt * p_velocity);
}

// ----------------------------------------------------------------------------
// TODO: si continuous wrap to -M_PIf to M_PI
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

    // Simply store the position value
    // Transform computation is now done in Robot::computeJointTransform()
    // and stored in State, not in the Blueprint
    m_position = p_position;
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
}

// ----------------------------------------------------------------------------
Transform const& Joint::localTransform() const
{
    // Return the static placement from URDF
    // Dynamic joint motion is handled in State, not here
    return m_local_transform;
}

// ----------------------------------------------------------------------------
void Joint::setIndex(size_t p_index)
{
    m_index = p_index;
}

} // namespace robotik