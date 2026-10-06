// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

//! @file Geometry.hpp
//! @brief Short names for the Compages geometry types, in double precision.
//!
//! Robotik defines no vector nor quaternion of its own: these are the
//! Compages types (rendering uses their float flavors).
#pragma once

#include "Compages/Core/Pose.hpp"
#include "Compages/Core/Units.hpp"

#include <cmath>

namespace robotik
{

//! @brief Position (m) or direction.
using Vector3 = compages::core::Vector3g;
//! @brief Unit quaternion (w, x, y, z).
using Quaternion = compages::core::Quatd;
//! @brief Frame placement in a parent frame (position in m).
using Pose = compages::core::Posed;

//! @brief Null vector (Compages vectors are left uninitialized by default).
inline Vector3 zero3()
{
    return Vector3(0.0, 0.0, 0.0);
}

//! @brief Euclidean norm.
inline double norm(Vector3 const& p_v)
{
    return compages::core::vector::norm(p_v);
}

//! @brief Roll about X, pitch about Y, yaw about Z (URDF convention).
[[nodiscard]] inline Quaternion rpy(double p_roll, double p_pitch, double p_yaw)
{
    return Quaternion::fromRpy(
        Radians(p_roll), Radians(p_pitch), Radians(p_yaw));
}

//! @brief Rotation whose columns are the given orthonormal axes.
[[nodiscard]] inline Quaternion
basis(Vector3 const& p_x, Vector3 const& p_y, Vector3 const& p_z)
{
    return Quaternion::fromAxes(p_x, p_y, p_z);
}

//! @brief Rotation of @p_angle rad about @p_axis.
[[nodiscard]] inline Quaternion axisAngle(Vector3 const& p_axis, double p_angle)
{
    double const length = norm(p_axis);
    if (length < 1e-12)
    {
        return {};
    }
    return Quaternion::rotation(Radians(p_angle),
                                p_axis.x / length,
                                p_axis.y / length,
                                p_axis.z / length);
}

//! @brief Heading about Z (rad).
[[nodiscard]] inline double yawOf(Quaternion const& p_q)
{
    return p_q.yaw().to<double>();
}

} // namespace robotik
