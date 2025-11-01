/**
 * @file State.cpp
 * @brief Implementation of State class.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#include "Robotik/Core/Robot/State.hpp"

namespace robotik
{

// ----------------------------------------------------------------------------
State::State(size_t p_nujoints, size_t p_nulinks)
{
    joint_positions.resize(p_nujoints, 0.0);
    joint_velocities.resize(p_nujoints, 0.0);
    joint_accelerations.resize(p_nujoints, 0.0);
    joint_local_transforms.resize(p_nujoints, Transform::Identity());
    joint_transforms.resize(p_nujoints, Transform::Identity());
    link_transforms.resize(p_nulinks, Transform::Identity());
    jacobian.resize(6, p_nujoints);
    jacobian.setZero();
}

} // namespace robotik
