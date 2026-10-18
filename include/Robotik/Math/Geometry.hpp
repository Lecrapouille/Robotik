// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file Geometry.hpp
/// @brief Short names for the Compages geometry types, in double precision.
///
/// Robotik defines no vector nor quaternion of its own: these are the
/// Compages types (rendering uses their float flavors).
#pragma once

#include "Compages/Core/Pose.hpp"
#include "Robotik/Math/Si.hpp"

#include <cmath>

namespace robotik
{

/// @brief Position (m) or direction.
using Vector3 = compages::core::Vector3g;
/// @brief Unit quaternion (w, x, y, z).
using Quaternion = compages::core::Quatd;
/// @brief Frame placement in a parent frame (position in m).
using Pose = compages::core::Posed;

/// @brief Null vector (Compages vectors are left uninitialized by default).
inline Vector3 zero3()
{
    return Vector3(0.0, 0.0, 0.0);
}

/// @brief Euclidean norm.
inline double norm(Vector3 const& p_v)
{
    return compages::core::vector::norm(p_v);
}

/// @brief Roll about X, pitch about Y, yaw about Z (URDF convention).
[[nodiscard]] inline Quaternion rpy(Radians p_roll, Radians p_pitch, Radians p_yaw)
{
    Quaternion const rx = Quaternion::rotation(p_roll, 1.0, 0.0, 0.0);
    Quaternion const ry = Quaternion::rotation(p_pitch, 0.0, 1.0, 0.0);
    Quaternion const rz = Quaternion::rotation(p_yaw, 0.0, 0.0, 1.0);
    return rz * ry * rx;
}

/// @brief Rotation whose columns are the given orthonormal axes.
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

/// @brief Rotation of @p_angle about @p_axis.
[[nodiscard]] inline Quaternion axisAngle(Vector3 const& p_axis, Radians p_angle)
{
    double const length = norm(p_axis);
    if (length < 1e-12)
    {
        return {};
    }
    return Quaternion::rotation(p_angle,
                                p_axis.x / length,
                                p_axis.y / length,
                                p_axis.z / length);
}

/// @brief Heading about Z (rad).
[[nodiscard]] inline double yawOf(Quaternion const& p_q)
{
    double const siny = 2.0 * (p_q.w * p_q.z + p_q.x * p_q.y);
    double const cosy = 1.0 - 2.0 * (p_q.y * p_q.y + p_q.z * p_q.z);
    return std::atan2(siny, cosy);
}

} // namespace robotik
