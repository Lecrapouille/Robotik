#pragma once

#include <Eigen/Dense>

namespace robotik
{

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
using Pose = Eigen::Matrix<double, 6, 1>;

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

} // namespace robotik