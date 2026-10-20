// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "main.hpp"

#include "Robotik/Task/Grasp/ContainerBounds.hpp"

using robotik::Vector3;
using robotik::ecs::Shape;
using robotik::scene::dropInto;
using robotik::scene::halfExtents;
using robotik::scene::innerBounds;
using robotik::scene::penetratesContainer;
using robotik::scene::restsInside;

// Same layout as data/scenarios/pick_and_place.scenario.yaml.
static Shape makeBox()
{
    Shape box;
    box.name = "box";
    box.type = Shape::Type::CONTAINER;
    box.size = { robotik::Length(0.16), robotik::Length(0.16), robotik::Length(0.08) };
    return box;
}

/// A 4 cm cube used as the grasped prop.
static Shape makeCube()
{
    Shape cube;
    cube.name = "red_cube";
    cube.type = Shape::Type::BOX;
    cube.size = { robotik::Length(0.04), robotik::Length(0.04), robotik::Length(0.04) };
    return cube;
}

static Vector3 const BOX_CENTER{ 0.40, -0.20, 0.04 };
static Vector3 const PICK_CENTER{ 0.40, 0.20, 0.02 };
static double constexpr SLACK = 0.05;

TEST(ContainerBounds, CavityMatchesTheRenderedWalls)
{
    std::optional<robotik::scene::ContainerInner> const inner =
        innerBounds(makeBox(), BOX_CENTER);
    ASSERT_TRUE(inner);
    EXPECT_NEAR(inner->half_x, 0.075, 1e-12);
    EXPECT_NEAR(inner->half_y, 0.075, 1e-12);
    EXPECT_NEAR(inner->floor_z, 0.005, 1e-12);
    EXPECT_NEAR(inner->rim_z, 0.08, 1e-12);
    EXPECT_FALSE(innerBounds(makeCube(), PICK_CENTER));
}

TEST(ContainerBounds, RestsInsideUsesRealExtentsAndTheRim)
{
    Shape const box = makeBox();
    Shape const cube = makeCube();
    Vector3 const half = halfExtents(cube);
    double const resting_z = 0.005 + half.z;

    EXPECT_TRUE(restsInside(box, BOX_CENTER, { 0.40, -0.20, resting_z }, half));

    // Centred in XY, still above the floor: not a placement.
    EXPECT_FALSE(restsInside(box, BOX_CENTER, { 0.40, -0.20, 0.05 }, half));

    // Carry height over the opening.
    EXPECT_FALSE(restsInside(box, BOX_CENTER, { 0.40, -0.20, 0.22 }, half));

    // Pick site of the scenario (40 cm from the box). The old 45 cm snap
    // would have counted this as a delivery.
    EXPECT_FALSE(restsInside(box, BOX_CENTER, PICK_CENTER, half));

    Shape slab = cube;
    slab.size[0] = robotik::Length(0.10);
    Vector3 const wide = halfExtents(slab);
    EXPECT_TRUE(
        restsInside(box, BOX_CENTER, { 0.40, -0.20, 0.005 + wide.z }, wide));
    EXPECT_FALSE(
        restsInside(box, BOX_CENTER, { 0.43, -0.20, 0.005 + wide.z }, wide));
}

TEST(ContainerBounds, DropStaysOnTheFootprint)
{
    Shape const box = makeBox();
    double constexpr HALF_Z = 0.02;
    double constexpr LIFT = 1e-4;

    std::optional<Vector3> const placed =
        dropInto(box, BOX_CENTER, { 0.40, -0.08, 0.15 }, HALF_Z, LIFT, SLACK);
    ASSERT_TRUE(placed);
    EXPECT_NEAR(placed->x, BOX_CENTER.x, 1e-12);
    EXPECT_NEAR(placed->y, BOX_CENTER.y, 1e-12);
    EXPECT_NEAR(placed->z, 0.005 + HALF_Z + LIFT, 1e-12);

    EXPECT_FALSE(dropInto(box, BOX_CENTER, PICK_CENTER, HALF_Z, LIFT, SLACK));
    EXPECT_FALSE(
        dropInto(box, BOX_CENTER, { 0.40, -0.06, 0.15 }, HALF_Z, LIFT, SLACK));
    EXPECT_FALSE(
        dropInto(makeCube(), PICK_CENTER, PICK_CENTER, HALF_Z, LIFT, SLACK));
}

TEST(ContainerBounds, PenetrationIsAWallCrossingBelowTheRim)
{
    Shape const box = makeBox();
    Vector3 const half = halfExtents(makeCube());
    Vector3 const through_wall{ 0.47, -0.20, 0.025 };

    EXPECT_TRUE(penetratesContainer(box, BOX_CENTER, through_wall, half));
    EXPECT_FALSE(restsInside(box, BOX_CENTER, through_wall, half));
    EXPECT_FALSE(
        penetratesContainer(box, BOX_CENTER, { 0.40, -0.20, 0.22 }, half));
    EXPECT_FALSE(penetratesContainer(box, BOX_CENTER, PICK_CENTER, half));
}
