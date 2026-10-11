// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

//! @file PainterScene.hpp
//! @brief Easel, free canvas and paint pots of the painter demo.
//!
//! The easel URDF is one rigid body (fixed joints) with a free joint at
//! @c easel_base, so it can slide or tip. The canvas is not in that file: the
//! simulation adds it as its own free box, 1 mm clear of the ledge. The robot
//! stays at the origin; the mast is 0.65 m in front of it (the skids reach
//! 0.45 m toward the robot). Frames here are URDF Z-up, the same frame MuJoCo
//! uses.
#pragma once

#include "Robotik/Math/Geometry.hpp"

#include <cmath>
#include <string>
#include <string_view>

namespace painter
{

inline constexpr double kEaselDistance = 0.65;
inline constexpr double kEaselFriction = 0.6;
inline constexpr double kFloorFriction = 0.8;
inline constexpr double kCanvasPitch = -0.1745;

inline constexpr char const* kEaselModel = "robotik.painter.easel";
inline constexpr char const* kCanvas = "canvas";
inline constexpr char const* kEaselRoot = "easel_base";

inline robotik::Vector3 cross(robotik::Vector3 const& p_a, robotik::Vector3 const& p_b)
{
    return { p_a.y * p_b.z - p_a.z * p_b.y,
             p_a.z * p_b.x - p_a.x * p_b.z,
             p_a.x * p_b.y - p_a.y * p_b.x };
}

inline robotik::Vector3 rotate(robotik::Quaternion const& p_q, robotik::Vector3 const& p_v)
{
    robotik::Vector3 const q(p_q.x, p_q.y, p_q.z);
    robotik::Vector3 const uv = cross(q, p_v);
    robotik::Vector3 const uuv = cross(q, uv);
    return { p_v.x + 2.0 * p_q.w * uv.x + 2.0 * uuv.x,
             p_v.y + 2.0 * p_q.w * uv.y + 2.0 * uuv.y,
             p_v.z + 2.0 * p_q.w * uv.z + 2.0 * uuv.z };
}

inline robotik::Pose compose(robotik::Pose const& p_parent, robotik::Pose const& p_local)
{
    return { p_parent.position + rotate(p_parent.rotation, p_local.position),
             p_parent.rotation * p_local.rotation };
}

//! Mast 0.65 m in front of the robot, painted face looking back at it.
inline robotik::Pose easelPose()
{
    return { { kEaselDistance, 0.0, 0.0 },
             robotik::rpy(Radians(0.0), Radians(0.0), Radians(std::numbers::pi)) };
}

//! Canvas centre in the easel base frame (1 mm off the ledge).
inline robotik::Pose canvasInEasel()
{
    return { { -0.1123, 0.0, 0.8893 },
             robotik::rpy(Radians(0.0), Radians(kCanvasPitch), Radians(0.0)) };
}

inline robotik::Pose canvasPose()
{
    return compose(easelPose(), canvasInEasel());
}

//! @c canvas_mount in the easel base: origin on the ledge, +x toward the robot
//! once the easel yaw is applied, +z up the canvas.
inline robotik::Pose canvasMountInEasel()
{
    return { { -0.0796, 0.0, 0.6402 },
             robotik::rpy(Radians(0.0), Radians(kCanvasPitch), Radians(0.0)) };
}

inline robotik::Vector3 canvasPoint(double p_x, double p_y, double p_z)
{
    robotik::Pose const mount = compose(easelPose(), canvasMountInEasel());
    return mount.position + rotate(mount.rotation, { p_x, p_y, p_z });
}

//! Brush pointing into the canvas (opposite the painted-face normal).
inline robotik::Quaternion brushAim()
{
    robotik::Pose const mount = compose(easelPose(), canvasMountInEasel());
    robotik::Vector3 const into = rotate(mount.rotation, { -1.0, 0.0, 0.0 });
    robotik::Vector3 const up = rotate(mount.rotation, { 0.0, 0.0, 1.0 });
    double const length = robotik::norm(into);
    robotik::Vector3 const z{ into.x / length, into.y / length, into.z / length };
    robotik::Vector3 x = cross(up, z);
    double const xl = robotik::norm(x);
    x = { x.x / xl, x.y / xl, x.z / xl };
    robotik::Vector3 const y = cross(z, x);
    return robotik::basis(x, y, z);
}

//! Tool straight down, for dipping into a pot.
inline robotik::Quaternion brushDown()
{
    return { 0.0, 0.0, 1.0, 0.0 };
}

//! @brief Shared with the mission so a stroke can tint the canvas texture.
struct Stroke
{
    std::string color = "red";
    bool painting = false;
};

inline int colorIndex(std::string_view p_name)
{
    if (p_name == "red")
    {
        return 0;
    }
    if (p_name == "black")
    {
        return 1;
    }
    if (p_name == "blue")
    {
        return 2;
    }
    return 3;
}

inline void colorRgb(std::string_view p_name, float& p_r, float& p_g, float& p_b)
{
    switch (colorIndex(p_name))
    {
        case 0:
            p_r = 0.82f;
            p_g = 0.18f;
            p_b = 0.18f;
            break;
        case 1:
            p_r = 0.08f;
            p_g = 0.08f;
            p_b = 0.10f;
            break;
        case 2:
            p_r = 0.10f;
            p_g = 0.22f;
            p_b = 0.62f;
            break;
        default:
            p_r = 0.86f;
            p_g = 0.67f;
            p_b = 0.16f;
            break;
    }
}

} // namespace painter
