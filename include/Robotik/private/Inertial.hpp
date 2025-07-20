#pragma once

#include "Robotik/private/Types.hpp"

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