/**
 * @file State.cpp
 * @brief Implementation of State class.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#include "Robotik/Core/State.hpp"

namespace robotik
{

// ----------------------------------------------------------------------------
State::State(size_t p_num_joints, size_t p_num_links)
{
    resize(p_num_joints, p_num_links);
}

// ----------------------------------------------------------------------------
void State::resize(size_t p_num_joints, size_t p_num_links)
{
    m_joint_positions.resize(p_num_joints, 0.0);
    m_joint_velocities.resize(p_num_joints, 0.0);
    m_joint_accelerations.resize(p_num_joints, 0.0);
    m_link_transforms.resize(p_num_links, Transform::Identity());
    m_joint_transforms.resize(p_num_joints, Transform::Identity());
    m_jacobian.resize(6, p_num_joints);
    m_jacobian.setZero();
    m_jacobian_dirty = true;
}

// ----------------------------------------------------------------------------
void State::setJointPositions(std::vector<double> const& p_positions)
{
    if (p_positions.size() != m_joint_positions.size())
    {
        return; // Size mismatch, ignore
    }
    m_joint_positions = p_positions;
    m_jacobian_dirty = true;
}

// ----------------------------------------------------------------------------
void State::setJointVelocities(std::vector<double> const& p_velocities)
{
    if (p_velocities.size() != m_joint_velocities.size())
    {
        return; // Size mismatch, ignore
    }
    m_joint_velocities = p_velocities;
}

// ----------------------------------------------------------------------------
void State::setJointAccelerations(std::vector<double> const& p_accelerations)
{
    if (p_accelerations.size() != m_joint_accelerations.size())
    {
        return; // Size mismatch, ignore
    }
    m_joint_accelerations = p_accelerations;
}

} // namespace robotik
