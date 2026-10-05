// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

//! @file Pose.hpp
//! @brief Small rigid-body math: 3D vectors, unit quaternions and poses.
//!
//! Plain aggregates of doubles with no Eigen in the public headers: they are
//! trivially copyable, sit contiguously in arrays and keep compile times low.
#pragma once

#include <cmath>

namespace robotik
{

// ****************************************************************************
//! @brief 3D vector in meters (or a direction when normalized).
// ****************************************************************************
struct Vector3
{
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    [[nodiscard]] constexpr Vector3 operator+(Vector3 const& p_other) const
    {
        return { x + p_other.x, y + p_other.y, z + p_other.z };
    }

    [[nodiscard]] constexpr Vector3 operator-(Vector3 const& p_other) const
    {
        return { x - p_other.x, y - p_other.y, z - p_other.z };
    }

    [[nodiscard]] constexpr Vector3 operator-() const
    {
        return { -x, -y, -z };
    }

    [[nodiscard]] constexpr Vector3 operator*(double p_scale) const
    {
        return { x * p_scale, y * p_scale, z * p_scale };
    }

    constexpr Vector3& operator+=(Vector3 const& p_other)
    {
        x += p_other.x;
        y += p_other.y;
        z += p_other.z;
        return *this;
    }

    [[nodiscard]] constexpr double dot(Vector3 const& p_other) const
    {
        return x * p_other.x + y * p_other.y + z * p_other.z;
    }

    [[nodiscard]] constexpr Vector3 cross(Vector3 const& p_other) const
    {
        return { y * p_other.z - z * p_other.y,
                 z * p_other.x - x * p_other.z,
                 x * p_other.y - y * p_other.x };
    }

    [[nodiscard]] double norm() const
    {
        return std::sqrt(dot(*this));
    }

    [[nodiscard]] Vector3 normalized() const
    {
        double const length = norm();
        return length > 0.0 ? *this * (1.0 / length) : Vector3{};
    }
};

// ****************************************************************************
//! @brief Unit quaternion, scalar first (w, x, y, z).
// ****************************************************************************
struct Quaternion
{
    double w = 1.0;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    // -------------------------------------------------------------------------
    //! @brief Rotation of @p_angle radians about @p_axis (normalized here).
    // -------------------------------------------------------------------------
    [[nodiscard]] static Quaternion axisAngle(Vector3 const& p_axis,
                                              double p_angle)
    {
        Vector3 const axis = p_axis.normalized();
        double const half = 0.5 * p_angle;
        double const s = std::sin(half);
        return { std::cos(half), axis.x * s, axis.y * s, axis.z * s };
    }

    // -------------------------------------------------------------------------
    //! @brief Roll about X, then pitch about Y, then yaw about Z (URDF rpy).
    // -------------------------------------------------------------------------
    [[nodiscard]] static Quaternion rpy(double p_roll,
                                        double p_pitch,
                                        double p_yaw)
    {
        return axisAngle({ 0.0, 0.0, 1.0 }, p_yaw) *
               axisAngle({ 0.0, 1.0, 0.0 }, p_pitch) *
               axisAngle({ 1.0, 0.0, 0.0 }, p_roll);
    }

    // -------------------------------------------------------------------------
    //! @brief Rotation whose matrix columns are the three given unit axes.
    // -------------------------------------------------------------------------
    [[nodiscard]] static Quaternion basis(Vector3 const& p_x,
                                          Vector3 const& p_y,
                                          Vector3 const& p_z)
    {
        double const trace = p_x.x + p_y.y + p_z.z;
        Quaternion q;
        if (trace > 0.0)
        {
            double const s = 2.0 * std::sqrt(trace + 1.0);
            q = { 0.25 * s,
                  (p_y.z - p_z.y) / s,
                  (p_z.x - p_x.z) / s,
                  (p_x.y - p_y.x) / s };
        }
        else if (p_x.x > p_y.y && p_x.x > p_z.z)
        {
            double const s = 2.0 * std::sqrt(1.0 + p_x.x - p_y.y - p_z.z);
            q = { (p_y.z - p_z.y) / s,
                  0.25 * s,
                  (p_y.x + p_x.y) / s,
                  (p_z.x + p_x.z) / s };
        }
        else if (p_y.y > p_z.z)
        {
            double const s = 2.0 * std::sqrt(1.0 + p_y.y - p_x.x - p_z.z);
            q = { (p_z.x - p_x.z) / s,
                  (p_y.x + p_x.y) / s,
                  0.25 * s,
                  (p_z.y + p_y.z) / s };
        }
        else
        {
            double const s = 2.0 * std::sqrt(1.0 + p_z.z - p_x.x - p_y.y);
            q = { (p_x.y - p_y.x) / s,
                  (p_z.x + p_x.z) / s,
                  (p_z.y + p_y.z) / s,
                  0.25 * s };
        }
        return q.normalized();
    }

    [[nodiscard]] constexpr Quaternion operator*(Quaternion const& p_q) const
    {
        return { w * p_q.w - x * p_q.x - y * p_q.y - z * p_q.z,
                 w * p_q.x + x * p_q.w + y * p_q.z - z * p_q.y,
                 w * p_q.y - x * p_q.z + y * p_q.w + z * p_q.x,
                 w * p_q.z + x * p_q.y - y * p_q.x + z * p_q.w };
    }

    [[nodiscard]] constexpr Quaternion conjugate() const
    {
        return { w, -x, -y, -z };
    }

    [[nodiscard]] Quaternion normalized() const
    {
        double const n = std::sqrt(w * w + x * x + y * y + z * z);
        return n > 0.0 ? Quaternion{ w / n, x / n, y / n, z / n }
                       : Quaternion{};
    }

    // -------------------------------------------------------------------------
    //! @brief Rotates @p_v by this (unit) quaternion.
    // -------------------------------------------------------------------------
    [[nodiscard]] constexpr Vector3 rotate(Vector3 const& p_v) const
    {
        Vector3 const u{ x, y, z };
        Vector3 const t = u.cross(p_v) * 2.0;
        return p_v + t * w + u.cross(t);
    }

    // -------------------------------------------------------------------------
    //! @brief Heading about Z in radians (ZYX convention).
    // -------------------------------------------------------------------------
    [[nodiscard]] double yaw() const
    {
        return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
    }
};

// ****************************************************************************
//! @brief Rigid transform: frame placement expressed in a parent frame.
//!
//! @c a * b chains placements (b expressed in a); @c pose * point maps a
//! point from the child frame to the parent frame.
// ****************************************************************************
struct Pose
{
    //!< Origin of the frame in the parent frame (SI: m).
    Vector3 position;
    //!< Orientation of the frame in the parent frame.
    Quaternion rotation;

    [[nodiscard]] constexpr Pose operator*(Pose const& p_child) const
    {
        return { position + rotation.rotate(p_child.position),
                 rotation * p_child.rotation };
    }

    [[nodiscard]] constexpr Vector3 operator*(Vector3 const& p_point) const
    {
        return position + rotation.rotate(p_point);
    }

    [[nodiscard]] constexpr Pose inverse() const
    {
        Quaternion const inverse = rotation.conjugate();
        return { inverse.rotate(-position), inverse };
    }
};

} // namespace robotik
