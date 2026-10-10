// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "FlyVision.hpp"

#include <algorithm>
#include <cmath>

namespace
{

constexpr double RAY_RANGE_M = 4.0;
constexpr double RAY_LEFT_RAD = 0.55;
constexpr double RAY_RIGHT_RAD = -0.55;

// neck_yaw origin in data/drosophila_x100.urdf, thorax frame.
constexpr double NECK_X_M = 0.07;
constexpr double NECK_Z_M = 0.015;

// Where each beam leaves the head, in the head frame: the two eye meshes
// and the front of the head. Left is +y.
struct EyePoint
{
    double x;
    double y;
    double z;
};

constexpr EyePoint EYE_POINTS[3] = {
    { 0.04, 0.032, 0.008 },
    { 0.08, 0.0, 0.012 },
    { 0.04, -0.032, 0.008 },
};

//! @brief Eye point from the head frame to the world: neck yaw, then body yaw.
robotik::Vector3
eyeOrigin(FlyPlant const& p_plant, EyePoint const& p_eye, double p_neck_yaw)
{
    double const neck_cos = std::cos(p_neck_yaw);
    double const neck_sin = std::sin(p_neck_yaw);
    double const head_x = neck_cos * p_eye.x - neck_sin * p_eye.y;
    double const head_y = neck_sin * p_eye.x + neck_cos * p_eye.y;
    double const body_x = NECK_X_M + head_x;
    double const body_y = head_y;
    double const body_z = NECK_Z_M + p_eye.z;
    double const yaw_cos = std::cos(p_plant.yaw);
    double const yaw_sin = std::sin(p_plant.yaw);
    return robotik::Vector3(p_plant.x + yaw_cos * body_x - yaw_sin * body_y,
                            p_plant.y + yaw_sin * body_x + yaw_cos * body_y,
                            p_plant.z + body_z);
}

//! @brief Slab method. Returns the entry distance, or past @p_range on a miss
//! and when the box is entirely behind the eye.
double rayBox(robotik::Vector3 const& p_origin,
              robotik::Vector3 const& p_direction,
              double p_range,
              FlyBox const& p_box)
{
    robotik::Vector3 const half(
        p_box.size.x * 0.5, p_box.size.y * 0.5, p_box.size.z * 0.5);
    double const origin[3] = { p_origin.x, p_origin.y, p_origin.z };
    double const direction[3] = { p_direction.x, p_direction.y, p_direction.z };
    double const lower[3] = { p_box.position.x - half.x,
                              p_box.position.y - half.y,
                              p_box.position.z - half.z };
    double const upper[3] = { p_box.position.x + half.x,
                              p_box.position.y + half.y,
                              p_box.position.z + half.z };
    double t_near = 0.0;
    double t_far = p_range;
    for (int axis = 0; axis < 3; ++axis)
    {
        if (std::fabs(direction[axis]) < 1e-9)
        {
            if (origin[axis] < lower[axis] || origin[axis] > upper[axis])
            {
                return p_range + 1.0;
            }
            continue;
        }
        double t1 = (lower[axis] - origin[axis]) / direction[axis];
        double t2 = (upper[axis] - origin[axis]) / direction[axis];
        if (t1 > t2)
        {
            std::swap(t1, t2);
        }
        t_near = std::max(t_near, t1);
        t_far = std::min(t_far, t2);
        if (t_near > t_far)
        {
            return p_range + 1.0;
        }
    }
    if (t_far < 0.0)
    {
        return p_range + 1.0;
    }
    return t_near;
}

//! @brief 1 at a contact, 0 at the end of the range or beyond it.
float channel(double p_distance, double p_range)
{
    if (p_distance > p_range)
    {
        return 0.0f;
    }
    return static_cast<float>(1.0 - p_distance / p_range);
}

} // namespace

FlyVision senseFly(FlyPlant const& p_plant,
                   std::span<FlyBox const> p_obstacles,
                   robotik::Random* p_noise,
                   double p_sigma,
                   double p_neck_yaw)
{
    // Calculate the angles for the three rays.
    double const angles[3] = { p_plant.yaw + RAY_LEFT_RAD,
                               p_plant.yaw,
                               p_plant.yaw + RAY_RIGHT_RAD };

    // Find the nearest obstacle.
    FlyVision vision;
    double nearest = RAY_RANGE_M;
    for (int ray = 0; ray < 3; ++ray)
    {
        // Calculate the origin and direction of the ray.
        robotik::Vector3 const origin =
            eyeOrigin(p_plant, EYE_POINTS[ray], p_neck_yaw);
        robotik::Vector3 const direction(
            std::cos(angles[ray]), std::sin(angles[ray]), 0.0);

        // Find the nearest obstacle.
        double hit = RAY_RANGE_M + 1.0;
        for (FlyBox const& box : p_obstacles)
        {
            hit = std::min(hit, rayBox(origin, direction, RAY_RANGE_M, box));
        }
        double const travel = std::min(hit, RAY_RANGE_M);

        // Write the origin and end of the ray.
        vision.origins[ray] = origin;
        vision.ends[ray] = robotik::Vector3(origin.x + direction.x * travel,
                                            origin.y + direction.y * travel,
                                            origin.z);

        // Calculate the value of the ray.
        float value = channel(hit, RAY_RANGE_M);

        // Add noise if requested.
        if (p_noise != nullptr && p_sigma > 0.0)
        {
            value += static_cast<float>(p_noise->normal(0.0, p_sigma));
            value = std::clamp(value, 0.0f, 1.0f);
        }

        // Write the value to the appropriate channel.
        if (ray == 0)
        {
            vision.left = value;
        }
        else if (ray == 1)
        {
            vision.center = value;
        }
        else
        {
            vision.right = value;
        }

        // Update the nearest distance if the ray hit an obstacle.
        if (hit <= RAY_RANGE_M)
        {
            nearest = std::min(nearest, hit);
        }
    }

    vision.distance = static_cast<float>(nearest);
    return vision;
}
