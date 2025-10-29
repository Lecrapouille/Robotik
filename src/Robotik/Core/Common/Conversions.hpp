/**
 * @file Conversions.hpp
 * @brief Conversion functions for transformations and poses.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#pragma once

#include "Robotik/Core/Common/Types.hpp"

namespace robotik
{

// ------------------------------------------------------------------------
//! \brief Convert Euler angles to rotation matrix.
//!
//! PHYSICS: Euler angles represent a sequence of rotations about coordinate
//! axes. This function implements the ZYX (yaw-pitch-roll) convention,
//! which is common in robotics.
//!
//! MATHEMATICAL FOUNDATION:
//! R = R_z(yaw) * R_y(pitch) * R_x(roll)
//! Where each R_i(θ) is a rotation matrix about axis i by angle θ
//!
//! ROTATION SEQUENCE:
//! 1. Roll (rx): Rotation about X-axis (bank angle)
//! 2. Pitch (ry): Rotation about Y-axis (elevation angle)
//! 3. Yaw (rz): Rotation about Z-axis (azimuth angle)
//!
//! GIMBAL LOCK WARNING:
//! This representation suffers from gimbal lock when pitch = ±π/2
//! For critical applications, consider using quaternions instead.
//!
//! \param p_rx Roll angle around X-axis (doubles)
//! \param p_ry Pitch angle around Y-axis (doubles)
//! \param p_rz Yaw angle around Z-axis (doubles)
//! \return 3x3 rotation matrix
// ------------------------------------------------------------------------
Eigen::Matrix3d eulerToRotation(double p_rx, double p_ry, double p_rz);

// ------------------------------------------------------------------------
//! \brief Extract Euler angles from rotation matrix.
//!
//! PHYSICS: Decomposes a 3x3 rotation matrix into three sequential
//! rotations about coordinate axes using ZYX convention.
//!
//! MATHEMATICAL PROCESS:
//! Given rotation matrix R, extract angles such that:
//! R = R_z(yaw) * R_y(pitch) * R_x(roll)
//!
//! SINGULARITY HANDLING:
//! - At gimbal lock (pitch = ±π/2), roll and yaw become interdependent
//! - Solution: Set roll to zero and compute yaw accordingly
//! - This ensures unique representation but may not preserve original angles
//!
//! ANGLE RANGES:
//! - Roll: [-π, π]
//! - Pitch: [-π/2, π/2]
//! - Yaw: [-π, π]
//!
//! \param p_rot 3x3 rotation matrix (must be orthogonal)
//! \return Vector containing [roll, pitch, yaw] angles
// ------------------------------------------------------------------------
Eigen::Vector3d rotationToEuler(const Eigen::Matrix3d& p_rot);

// ------------------------------------------------------------------------
//! \brief Create homogeneous transformation from translation and rotation.
//!
//! PHYSICS: Combines separate translation and rotation into a single
//! 4x4 homogeneous transformation matrix for efficient computation.
//!
//! MATHEMATICAL STRUCTURE:
//! T = [R  t]  where R is 3x3 rotation matrix, t is 3x1 translation
//!     [0  1]
//!
//! GEOMETRIC INTERPRETATION:
//! - First applies rotation R to a point
//! - Then adds translation t
//! - Composition: T(p) = R*p + t
//!
//! \param p_translation 3D translation vector [x, y, z]
//! \param p_rotation 3x3 rotation matrix
//! \return 4x4 homogeneous transformation matrix
// ------------------------------------------------------------------------
Transform createTransform(const Eigen::Vector3d& p_translation,
                          const Eigen::Matrix3d& p_rotation);

// ------------------------------------------------------------------------
//! \brief Create homogeneous transformation from translation and Euler angles.
//!
//! PHYSICS: Convenience function that combines translation with rotation
//! specified as Euler angles using ZYX convention.
//!
//! PROCESS:
//! 1. Convert Euler angles to rotation matrix
//! 2. Combine with translation into homogeneous form
//!
//! APPLICATIONS:
//! - Robot link transformations
//! - Camera pose specification
//! - Object placement in 3D space
//!
//! \param p_translation 3D translation vector [x, y, z]
//! \param p_rx Roll angle around X-axis (doubles)
//! \param p_ry Pitch angle around Y-axis (doubles)
//! \param p_rz Yaw angle around Z-axis (doubles)
//! \return 4x4 homogeneous transformation matrix
// ------------------------------------------------------------------------
Transform createTransform(const Eigen::Vector3d& p_translation,
                          double p_rx,
                          double p_ry,
                          double p_rz);

// Extraction of data from transformations
Eigen::Vector3d getTranslation(Transform const& p_transform);
Eigen::Matrix3d getRotation(Transform const& p_transform);
Pose transformToPose(Transform const& p_transform);
Transform poseToTransform(Pose const& p_pose);

// ------------------------------------------------------------------------
//! \brief Create transformation matrix using Denavit-Hartenberg parameters.
//!
//! PHYSICS: The Denavit-Hartenberg (DH) convention is a systematic method
//! for describing the geometry of serial robotic manipulators. It provides
//! a standardized way to represent the relationship between consecutive
//! joint coordinate frames.
//!
//! DH PARAMETERS:
//! - a (link length): Distance between joint axes along common normal
//! - α (link twist): Angle between joint axes about common normal
//! - d (link offset): Distance between common normals along joint axis
//! - θ (joint angle): Angle between common normals about joint axis
//!
//! TRANSFORMATION SEQUENCE:
//! 1. Rotate about Z-axis by θ (joint angle)
//! 2. Translate along Z-axis by d (link offset)
//! 3. Translate along X-axis by a (link length)
//! 4. Rotate about X-axis by α (link twist)
//!
//! MATHEMATICAL FORMULA:
//! T = Rot_z(θ) * Trans_z(d) * Trans_x(a) * Rot_x(α)
//!
//! APPLICATIONS:
//! - Forward kinematics computation
//! - Robot modeling and simulation
//! - Kinematic calibration
//! - Workspace analysis
//!
//! \param p_a Link length parameter (meters)
//! \param p_alpha Link twist parameter (doubles)
//! \param p_d Link offset parameter (meters)
//! \param p_theta Joint angle parameter (doubles)
//! \return 4x4 homogeneous transformation matrix
// ------------------------------------------------------------------------
Transform dhTransform(double p_a, double p_alpha, double p_d, double p_theta);

// ------------------------------------------------------------------------
//! \brief Print a transformation matrix.
//! \param p_transform The transformation matrix to print.
//! \param p_precision The number of decimal places to print.
//! \return The string representation of the transformation matrix.
// ------------------------------------------------------------------------
std::string printTransform(const Transform& p_transform, int p_precision = 3);

// ------------------------------------------------------------------------
//! \brief Print a pose.
//! \param p_name The name of the pose.
//! \param p_pose The pose to print.
//! \return The string representation of the pose.
// ------------------------------------------------------------------------
std::string printPose(const std::string& name, const Pose& pose);

// ------------------------------------------------------------------------
//! \brief Calculate the pose error between a target and a current pose.
//! \param p_target The target pose.
//! \param p_current The current pose.
//! \return The pose error.
// ------------------------------------------------------------------------
Pose calculatePoseError(Pose const& p_target, Pose const& p_current);

} // namespace robotik