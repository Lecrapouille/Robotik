/**
 * @file Inertial.hpp
 * @brief Inertial properties used for a robotic link.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#pragma once

#include "Robotik/Core/Types.hpp"

namespace robotik
{

// ****************************************************************************
//! \brief Inertial properties of a robotic link.
// ****************************************************************************
struct Inertial
{
    Inertial()
    {
        center_of_mass = Eigen::Vector3d::Zero();
        inertia_matrix = Eigen::Matrix3d::Identity();
    }

    //! \brief Mass of the object (kg).
    double mass = 0.0;
    //! \brief Center of mass of the object (m).
    Eigen::Vector3d center_of_mass;
    //! \brief Inertia matrix of the object.
    Eigen::Matrix3d inertia_matrix;
};

} // namespace robotik