// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

//! @file ContainerBounds.hpp
//! @brief Axis-aligned inner cavity of open box containers (@ref ecs::SceneObject::Type::BOX).
//!
//! @par Scope
//! Helpers for scenario props whose @ref ecs::SceneObject::Type is @c BOX: an
//! open-top bin (floor slab + four walls). Not arbitrary meshes, not MuJoCo
//! collision shapes.
//!
//! @par Why @c Scene/ and not @c ECS/
//! @ref ecs::SceneObject is an ECS component (data on entities). This file is
//! layout math on that component: cavity size, placement checks, penetration
//! while carried. Same rules drive rendering (@ref SimulatorView) and
//! kinematic props (@ref GraspSystem) without duplicating wall thickness.
//!
//! @par Coordinates
//! Robot base frame (entities parented to the robot root). Metres.
//!
//! @par Example (placement assert)
//! @code
//! ecs::SceneObject const& box = container.get<ecs::SceneObject>();
//! ecs::SceneObject const& cube = object.get<ecs::SceneObject>();
//! bool ok = scene::restsInside(box, box_center, cube_center, scene::halfExtents(cube));
//! @endcode
//!
//! @par Example (carried cube vs walls)
//! @code
//! if (scene::penetratesContainer(box, box_center, cube_center, cube_half))
//! {
//!     // Count toward collisions == 0 while the vacuum holds the cube.
//! }
//! @endcode
#pragma once

#include "Robotik/ECS/ObjectComponents.hpp"
#include "Robotik/Math/Geometry.hpp"

#include <optional>

namespace robotik::scene
{

//! @brief Inner cavity of a @ref ecs::SceneObject::Type::BOX container.
struct ContainerInner
{
    Vector3 center{};    //!< XY centre of the cavity (entity position).
    double half_x = 0.0; //!< Half-width along X inside the walls.
    double half_y = 0.0; //!< Half-width along Y inside the walls.
    double floor_z = 0.0; //!< Z of the top surface of the inner floor slab.
    double rim_z = 0.0;   //!< Z of the top of the walls.
};

[[nodiscard]] std::optional<ContainerInner>
innerBounds(ecs::SceneObject const& p_object, Vector3 const& p_center);

[[nodiscard]] bool insideInner(ContainerInner const& p_inner,
                               Vector3 const& p_point,
                               double p_margin = 0.0);

[[nodiscard]] bool overOuterFootprint(ecs::SceneObject const& p_object,
                                      Vector3 const& p_center,
                                      Vector3 const& p_point,
                                      double p_slack = 0.0);

[[nodiscard]] bool fitsInside(ContainerInner const& p_inner,
                              Vector3 const& p_point,
                              Vector3 const& p_half);

//! @brief Half extents (m) of a scenario prop.
[[nodiscard]] inline Vector3 halfExtents(ecs::SceneObject const& p_object)
{
    return { 0.5 * p_object.size[0].value(),
             0.5 * p_object.size[1].value(),
             0.5 * p_object.size[2].value() };
}

//! @brief True when the object sits on the inner floor, within the walls and
//! below the rim. Shared by scenario asserts and the pick-and-place reward.
[[nodiscard]] bool restsInside(ecs::SceneObject const& p_container,
                               Vector3 const& p_container_center,
                               Vector3 const& p_object_center,
                               Vector3 const& p_object_half);

[[nodiscard]] bool penetratesContainer(ecs::SceneObject const& p_container,
                                     Vector3 const& p_container_center,
                                     Vector3 const& p_cube_center,
                                     Vector3 const& p_cube_half);

//! @brief Centre of the cavity, on the floor, when @p_point lies over the outer
//! footprint expanded by @p_slack (m). Empty otherwise.
[[nodiscard]] std::optional<Vector3>
dropInto(ecs::SceneObject const& p_container,
         Vector3 const& p_container_center,
         Vector3 const& p_point,
         double p_half_z,
         double p_lift,
         double p_slack);

} // namespace robotik::scene
