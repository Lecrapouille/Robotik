// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file ContainerBounds.hpp
/// Axis-aligned inner cavity of open box containers (@ref
/// ecs::Shape::Type::CONTAINER).
///
/// @par Scope
/// Helpers for scenario props whose @ref ecs::Shape::Type is @c CONTAINER: an
/// open-top bin (floor slab + four walls). Not arbitrary meshes, not MuJoCo
/// collision shapes.
///
/// @par Why @c Scene/ and not @c ECS/
/// @ref ecs::Shape is an ECS component (data on entities). This file is
/// layout math on that component: cavity size, placement checks, penetration
/// while carried. Same rules drive rendering (@ref SimulatorView) and
/// kinematic props (@ref AbstractGrasp) without duplicating wall thickness.
///
/// @par Coordinates
/// Robot base frame (entities parented to the robot root). Metres.
///
/// @par Example (placement assert)
/// @code
/// ecs::Shape const& box = container.get<ecs::Shape>();
/// ecs::Shape const& cube = object.get<ecs::Shape>();
/// bool ok = scene::restsInside(box, box_center, cube_center,
/// scene::halfExtents(cube));
/// @endcode
///
/// @par Example (carried cube vs walls)
/// @code
/// if (scene::penetratesContainer(box, box_center, cube_center, cube_half))
/// {
///     // Count toward collisions == 0 while the vacuum holds the cube.
/// }
/// @endcode
#pragma once

#include "Robotik/Robot/Components/Objects.hpp"
#include "Robotik/Common/Math.hpp"

#include <optional>

namespace robotik::scene
{

// ****************************************************************************
/// Inner cavity of a @ref ecs::Shape::Type::CONTAINER container: the free
/// space inside the walls and above the floor slab.
///
/// @code
/// if (auto const inner = scene::innerBounds(box, box_center))
///     std::cout << "cavity " << 2.0 * inner->half_x << " x "
///               << 2.0 * inner->half_y << " m\n";
/// @endcode
// ****************************************************************************
struct ContainerInner
{
    Vector3 center{};     ///< XY centre of the cavity (entity position).
    double half_x = 0.0;  ///< Half-width along X inside the walls.
    double half_y = 0.0;  ///< Half-width along Y inside the walls.
    double floor_z = 0.0; ///< Z of the top surface of the inner floor slab.
    double rim_z = 0.0;   ///< Z of the top of the walls.
};

// ----------------------------------------------------------------------------
/// Computes the cavity of a container.
/// @param p_object Shape of the container.
/// @param p_center Centre of the container.
/// @return The cavity, or nothing when @p_object is not a container.
// ----------------------------------------------------------------------------
[[nodiscard]] std::optional<ContainerInner>
innerBounds(ecs::Shape const& p_object, Vector3 const& p_center);

// ----------------------------------------------------------------------------
/// Tells whether a point is inside the walls, seen from above (Z is not
/// checked).
/// @param p_inner Cavity of the container.
/// @param p_point Point to test.
/// @param p_margin Distance (m) the point must keep from the walls.
/// @return True when the point is within the walls minus the margin.
// ----------------------------------------------------------------------------
[[nodiscard]] bool insideInner(ContainerInner const& p_inner,
                               Vector3 const& p_point,
                               double p_margin = 0.0);

// ----------------------------------------------------------------------------
/// Tells whether a point is above the outer footprint of an object, seen
/// from above (Z is not checked).
/// @param p_object Shape of the object.
/// @param p_center Centre of the object.
/// @param p_point Point to test.
/// @param p_slack Distance (m) added around the footprint.
/// @return True when the point is over the footprint plus the slack.
// ----------------------------------------------------------------------------
[[nodiscard]] bool overOuterFootprint(ecs::Shape const& p_object,
                                      Vector3 const& p_center,
                                      Vector3 const& p_point,
                                      double p_slack = 0.0);

// ----------------------------------------------------------------------------
/// Tells whether a box fits in the cavity: within the walls and not below
/// the floor.
/// @param p_inner Cavity of the container.
/// @param p_point Centre of the box.
/// @param p_half Half extents of the box (m).
/// @return True when the whole box is inside the walls and above the floor.
// ----------------------------------------------------------------------------
[[nodiscard]] bool fitsInside(ContainerInner const& p_inner,
                              Vector3 const& p_point,
                              Vector3 const& p_half);

// ----------------------------------------------------------------------------
/// Half extents of a scenario prop.
/// @param p_object Shape of the prop.
/// @return Half of its size along X, Y and Z (m).
// ----------------------------------------------------------------------------
[[nodiscard]] inline Vector3 halfExtents(ecs::Shape const& p_object)
{
    return { 0.5 * p_object.size[0].value(),
             0.5 * p_object.size[1].value(),
             0.5 * p_object.size[2].value() };
}

// ----------------------------------------------------------------------------
/// Tells whether an object was put down in a container. Shared by scenario
/// asserts and the pick-and-place reward.
/// @param p_container Shape of the container.
/// @param p_container_center Centre of the container.
/// @param p_object_center Centre of the object.
/// @param p_object_half Half extents of the object (m).
/// @return True when the object sits on the inner floor, within the walls
///   and below the rim.
// ----------------------------------------------------------------------------
[[nodiscard]] bool restsInside(ecs::Shape const& p_container,
                               Vector3 const& p_container_center,
                               Vector3 const& p_object_center,
                               Vector3 const& p_object_half);

// ----------------------------------------------------------------------------
/// Tells whether a carried cube goes through the walls of a container.
/// @param p_container Shape of the container.
/// @param p_container_center Centre of the container.
/// @param p_cube_center Centre of the cube.
/// @param p_cube_half Half extents of the cube (m).
/// @return True when the cube is over the container, lower than the rim,
///   and does not fit in the cavity. False when @p_container is not a
///   container.
// ----------------------------------------------------------------------------
[[nodiscard]] bool penetratesContainer(ecs::Shape const& p_container,
                                       Vector3 const& p_container_center,
                                       Vector3 const& p_cube_center,
                                       Vector3 const& p_cube_half);

// ----------------------------------------------------------------------------
/// Where to put an object released over a container.
/// @param p_container Shape of the container.
/// @param p_container_center Centre of the container.
/// @param p_point Point where the object was released.
/// @param p_half_z Half height of the object (m).
/// @param p_lift Extra height (m) above the floor, to avoid a contact at
///   once.
/// @param p_slack Distance (m) added around the outer footprint.
/// @return Centre of the object resting in the middle of the cavity, when
///   @p_point lies over the footprint plus the slack. Empty otherwise.
// ----------------------------------------------------------------------------
[[nodiscard]] std::optional<Vector3> dropInto(ecs::Shape const& p_container,
                                              Vector3 const& p_container_center,
                                              Vector3 const& p_point,
                                              double p_half_z,
                                              double p_lift,
                                              double p_slack);

} // namespace robotik::scene
