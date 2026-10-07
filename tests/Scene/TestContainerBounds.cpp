// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "main.hpp"

#include "Robotik/Scene/ContainerBounds.hpp"

using robotik::Vector3;
using robotik::ecs::SceneObject;
using robotik::scene::dropInto;
using robotik::scene::halfExtents;
using robotik::scene::innerBounds;
using robotik::scene::penetratesContainer;
using robotik::scene::restsInside;

namespace
{

// Same layout as data/scenarios/pick_and_place.yml.
SceneObject makeBox()
{
    SceneObject box;
    box.name = "box";
    box.type = SceneObject::Type::BOX;
    box.size = { Length(0.16), Length(0.16), Length(0.08) };
    return box;
}

SceneObject makeCube()
{
    SceneObject cube;
    cube.name = "red_cube";
    cube.type = SceneObject::Type::CUBE;
    cube.size = { Length(0.04), Length(0.04), Length(0.04) };
    return cube;
}

Vector3 const kBoxCenter{ 0.40, -0.20, 0.04 };
Vector3 const kPickCenter{ 0.40, 0.20, 0.02 };
double constexpr kSlack = 0.05;

} // namespace

TEST(ContainerBounds, CavityMatchesTheRenderedWalls)
{
    std::optional<robotik::scene::ContainerInner> const inner =
        innerBounds(makeBox(), kBoxCenter);
    ASSERT_TRUE(inner);
    EXPECT_NEAR(inner->half_x, 0.075, 1e-12);
    EXPECT_NEAR(inner->half_y, 0.075, 1e-12);
    EXPECT_NEAR(inner->floor_z, 0.005, 1e-12);
    EXPECT_NEAR(inner->rim_z, 0.08, 1e-12);
    EXPECT_FALSE(innerBounds(makeCube(), kPickCenter));
}

TEST(ContainerBounds, RestsInsideUsesRealExtentsAndTheRim)
{
    SceneObject const box = makeBox();
    SceneObject const cube = makeCube();
    Vector3 const half = halfExtents(cube);
    double const resting_z = 0.005 + half.z;

    EXPECT_TRUE(restsInside(box, kBoxCenter, { 0.40, -0.20, resting_z }, half));

    // Centred in XY, still above the floor: not a placement.
    EXPECT_FALSE(restsInside(box, kBoxCenter, { 0.40, -0.20, 0.05 }, half));

    // Carry height over the opening.
    EXPECT_FALSE(restsInside(box, kBoxCenter, { 0.40, -0.20, 0.22 }, half));

    // Pick site of the scenario (40 cm from the box). The old 45 cm snap
    // would have counted this as a delivery.
    EXPECT_FALSE(restsInside(box, kBoxCenter, kPickCenter, half));

    SceneObject slab = cube;
    slab.size[0] = Length(0.10);
    Vector3 const wide = halfExtents(slab);
    EXPECT_TRUE(restsInside(
        box, kBoxCenter, { 0.40, -0.20, 0.005 + wide.z }, wide));
    EXPECT_FALSE(restsInside(
        box, kBoxCenter, { 0.43, -0.20, 0.005 + wide.z }, wide));
}

TEST(ContainerBounds, DropStaysOnTheFootprint)
{
    SceneObject const box = makeBox();
    double constexpr kHalfZ = 0.02;
    double constexpr kLift = 1e-4;

    std::optional<Vector3> const placed =
        dropInto(box, kBoxCenter, { 0.40, -0.08, 0.15 }, kHalfZ, kLift, kSlack);
    ASSERT_TRUE(placed);
    EXPECT_NEAR(placed->x, kBoxCenter.x, 1e-12);
    EXPECT_NEAR(placed->y, kBoxCenter.y, 1e-12);
    EXPECT_NEAR(placed->z, 0.005 + kHalfZ + kLift, 1e-12);

    EXPECT_FALSE(
        dropInto(box, kBoxCenter, kPickCenter, kHalfZ, kLift, kSlack));
    EXPECT_FALSE(dropInto(
        box, kBoxCenter, { 0.40, -0.06, 0.15 }, kHalfZ, kLift, kSlack));
    EXPECT_FALSE(
        dropInto(makeCube(), kPickCenter, kPickCenter, kHalfZ, kLift, kSlack));
}

TEST(ContainerBounds, PenetrationIsAWallCrossingBelowTheRim)
{
    SceneObject const box = makeBox();
    Vector3 const half = halfExtents(makeCube());
    Vector3 const through_wall{ 0.47, -0.20, 0.025 };

    EXPECT_TRUE(penetratesContainer(box, kBoxCenter, through_wall, half));
    EXPECT_FALSE(restsInside(box, kBoxCenter, through_wall, half));
    EXPECT_FALSE(
        penetratesContainer(box, kBoxCenter, { 0.40, -0.20, 0.22 }, half));
    EXPECT_FALSE(
        penetratesContainer(box, kBoxCenter, kPickCenter, half));
}
