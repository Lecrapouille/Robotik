// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file Geometry.hpp
/// Short names for the Compages geometry types, in double precision.
///
/// Robotik defines no vector nor quaternion of its own: these are the
/// Compages types (rendering uses their float flavors).
///
/// @code
/// robotik::Pose tool;
/// tool.position = robotik::Vector3(0.3, 0.0, 0.2);
/// tool.rotation = robotik::rpy(Radians(0.0), Radians(1.5708), Radians(0.0));
/// double heading = robotik::yawOf(tool.rotation);
/// @endcode
#pragma once

#include "Compages/Core/Pose.hpp"
#include "Robotik/Math/Si.hpp"

#include <cmath>

namespace robotik
{

/// Position (m) or direction.
using Vector3 = compages::core::Vector3g;
/// Unit quaternion (w, x, y, z).
using Quaternion = compages::core::Quatd;
/// Frame placement in a parent frame (position in m).
using Pose = compages::core::Posed;

// ----------------------------------------------------------------------------
/// Null vector. Use it instead of @c Vector3(): Compages vectors are left
/// uninitialized by default.
/// @return (0, 0, 0).
// ----------------------------------------------------------------------------
inline Vector3 zero3()
{
    return Vector3(0.0, 0.0, 0.0);
}

// ----------------------------------------------------------------------------
/// Euclidean norm (length) of a vector.
/// @param p_v Vector to measure.
/// @return sqrt(x² + y² + z²).
// ----------------------------------------------------------------------------
inline double norm(Vector3 const& p_v)
{
    return compages::core::vector::norm(p_v);
}

// ----------------------------------------------------------------------------
/// Rotation from roll, pitch and yaw (URDF convention): roll about X first,
/// then pitch about Y, then yaw about Z, all about the fixed axes.
/// @param p_roll Angle about X.
/// @param p_pitch Angle about Y.
/// @param p_yaw Angle about Z.
/// @return The rotation Rz * Ry * Rx.
// ----------------------------------------------------------------------------
[[nodiscard]] inline Quaternion
rpy(Radians p_roll, Radians p_pitch, Radians p_yaw)
{
    Quaternion const rx = Quaternion::rotation(p_roll, 1.0, 0.0, 0.0);
    Quaternion const ry = Quaternion::rotation(p_pitch, 0.0, 1.0, 0.0);
    Quaternion const rz = Quaternion::rotation(p_yaw, 0.0, 0.0, 1.0);
    return rz * ry * rx;
}

// ----------------------------------------------------------------------------
/// Rotation whose columns are the given orthonormal axes: the rotated frame
/// has its X along @p_x, its Y along @p_y and its Z along @p_z.
/// @param p_x Unit X axis of the new frame.
/// @param p_y Unit Y axis, orthogonal to @p_x.
/// @param p_z Unit Z axis, orthogonal to both.
/// @return The rotation.
// ----------------------------------------------------------------------------
[[nodiscard]] inline Quaternion
basis(Vector3 const& p_x, Vector3 const& p_y, Vector3 const& p_z)
{
    compages::core::Matrix<double, 4u, 4u> m(compages::core::matrix::Identity);
    m(0, 0) = p_x.x;
    m(1, 0) = p_x.y;
    m(2, 0) = p_x.z;
    m(0, 1) = p_y.x;
    m(1, 1) = p_y.y;
    m(2, 1) = p_y.z;
    m(0, 2) = p_z.x;
    m(1, 2) = p_z.y;
    m(2, 2) = p_z.z;
    return Quaternion::fromMatrix(m);
}

// ----------------------------------------------------------------------------
/// Rotation of an angle about an axis.
/// @param p_axis Rotation axis; it does not need to be of length 1.
/// @param p_angle Angle, positive by the right-hand rule.
/// @return The rotation, or the identity when the axis is (almost) null.
// ----------------------------------------------------------------------------
[[nodiscard]] inline Quaternion axisAngle(Vector3 const& p_axis,
                                          Radians p_angle)
{
    double const length = norm(p_axis);
    if (length < 1e-12)
    {
        return {};
    }
    return Quaternion::rotation(
        p_angle, p_axis.x / length, p_axis.y / length, p_axis.z / length);
}

// ----------------------------------------------------------------------------
/// Heading about Z: where the X axis of the rotated frame points in the
/// horizontal plane.
/// @param p_q Rotation to read.
/// @return The yaw angle in (-pi, pi] (rad).
// ----------------------------------------------------------------------------
[[nodiscard]] inline double yawOf(Quaternion const& p_q)
{
    double const siny = 2.0 * (p_q.w * p_q.z + p_q.x * p_q.y);
    double const cosy = 1.0 - 2.0 * (p_q.y * p_q.y + p_q.z * p_q.z);
    return std::atan2(siny, cosy);
}

} // namespace robotik
