/**
 * @file Types.hpp
 * @brief Types for the Robotik library.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#pragma once

#include <Eigen/Dense>

namespace robotik
{

// ----------------------------------------------------------------------------
//! \brief Vector of values for each joint at a given time (i.e. positions,
//! velocities, accelerations).
// ----------------------------------------------------------------------------
using JointValues = std::vector<double>;

// ----------------------------------------------------------------------------
//! \brief Homogeneous transformation matrix (4x4) for 3D spatial
//! transformations.
//!
//! This 4x4 matrix encodes both rotation and translation in a single
//! representation: [R t] where R is 3x3 rotation matrix, t is 3x1 translation
//! vector [0 1] bottom row is always [0 0 0 1] for homogeneous coordinates
//!
//! Used for: Forward kinematics, coordinate frame transformations, pose
//! composition.
// ----------------------------------------------------------------------------
using Transform = Eigen::Matrix4d;

// ----------------------------------------------------------------------------
//! \brief 6D pose vector representing position and orientation
//! (x,y,z,rx,ry,rz).
//!
//! This compact representation combines:
//! - Position: [x,y,z] - 3D coordinates in meters
//! - Orientation: [rx,ry,rz] - Euler angles in radians (roll, pitch, yaw)
//!
//! Used for: End-effector targets, trajectory waypoints, inverse kinematics
//! goals.
// ----------------------------------------------------------------------------
using Pose = Eigen::Matrix<double, 6, 1>; // Eigen::Affine3d

// ----------------------------------------------------------------------------
//! \brief Jacobian matrix relating joint velocities to end-effector velocity.
//!
//! This matrix (typically 6×n for n joints) provides the linear relationship:
//! end_effector_velocity = J * joint_velocities
//!
//! Used for: Velocity control, force control, singularity analysis, inverse
//! kinematics
// ----------------------------------------------------------------------------
using Jacobian = Eigen::MatrixXd;

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