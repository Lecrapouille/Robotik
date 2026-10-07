// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "Robotik/Scene/ContainerBounds.hpp"

#include <algorithm>
#include <cmath>

namespace robotik::scene
{

static constexpr double kFloorContactEpsilonM = 1e-3;

std::optional<ContainerInner> innerBounds(ecs::SceneObject const& p_object,
                                          Vector3 const& p_center)
{
    if (p_object.type != ecs::SceneObject::Type::BOX)
    {
        return std::nullopt;
    }
    double const half_z = 0.5 * p_object.size[2].value();
    ContainerInner inner;
    inner.center = p_center;
    inner.half_x =
        std::max(0.0, 0.5 * p_object.size[0].value() - ecs::CONTAINER_WALL_M);
    inner.half_y =
        std::max(0.0, 0.5 * p_object.size[1].value() - ecs::CONTAINER_WALL_M);
    inner.floor_z = p_center.z - half_z + ecs::CONTAINER_WALL_M;
    inner.rim_z = p_center.z + half_z;
    return inner;
}

bool insideInner(ContainerInner const& p_inner,
                 Vector3 const& p_point,
                 double p_margin)
{
    return std::abs(p_point.x - p_inner.center.x) <= p_inner.half_x - p_margin &&
           std::abs(p_point.y - p_inner.center.y) <= p_inner.half_y - p_margin;
}

bool overOuterFootprint(ecs::SceneObject const& p_object,
                        Vector3 const& p_center,
                        Vector3 const& p_point,
                        double p_slack)
{
    double const hx = 0.5 * p_object.size[0].value() + p_slack;
    double const hy = 0.5 * p_object.size[1].value() + p_slack;
    return std::abs(p_point.x - p_center.x) <= hx &&
           std::abs(p_point.y - p_center.y) <= hy;
}

bool fitsInside(ContainerInner const& p_inner,
                Vector3 const& p_point,
                Vector3 const& p_half)
{
    return p_point.x - p_half.x >= p_inner.center.x - p_inner.half_x &&
           p_point.x + p_half.x <= p_inner.center.x + p_inner.half_x &&
           p_point.y - p_half.y >= p_inner.center.y - p_inner.half_y &&
           p_point.y + p_half.y <= p_inner.center.y + p_inner.half_y &&
           p_point.z - p_half.z >= p_inner.floor_z - kFloorContactEpsilonM;
}

bool restsInside(ecs::SceneObject const& p_container,
                 Vector3 const& p_container_center,
                 Vector3 const& p_object_center,
                 Vector3 const& p_object_half)
{
    std::optional<ContainerInner> const inner =
        innerBounds(p_container, p_container_center);
    if (!inner)
    {
        return false;
    }
    double const bottom = p_object_center.z - p_object_half.z;
    double const top = p_object_center.z + p_object_half.z;
    return fitsInside(*inner, p_object_center, p_object_half) &&
           top <= inner->rim_z + kFloorContactEpsilonM &&
           bottom <= inner->floor_z + kFloorContactEpsilonM;
}

bool penetratesContainer(ecs::SceneObject const& p_container,
                         Vector3 const& p_container_center,
                         Vector3 const& p_cube_center,
                         Vector3 const& p_cube_half)
{
    if (p_container.type != ecs::SceneObject::Type::BOX)
    {
        return false;
    }
    std::optional<ContainerInner> const inner =
        innerBounds(p_container, p_container_center);
    if (!inner)
    {
        return false;
    }
    if (!overOuterFootprint(
            p_container, p_container_center, p_cube_center, 0.0))
    {
        return false;
    }
    if (p_cube_center.z - p_cube_half.z > inner->rim_z + kFloorContactEpsilonM)
    {
        return false;
    }
    return !fitsInside(*inner, p_cube_center, p_cube_half);
}

std::optional<Vector3> dropInto(ecs::SceneObject const& p_container,
                                Vector3 const& p_container_center,
                                Vector3 const& p_point,
                                double p_half_z,
                                double p_lift,
                                double p_slack)
{
    std::optional<ContainerInner> const inner =
        innerBounds(p_container, p_container_center);
    if (!inner ||
        !overOuterFootprint(
            p_container, p_container_center, p_point, p_slack))
    {
        return std::nullopt;
    }
    return Vector3{ inner->center.x,
                    inner->center.y,
                    inner->floor_z + p_half_z + p_lift };
}

} // namespace robotik::scene
