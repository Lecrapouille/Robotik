// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "main.hpp"

#include "Robotik/Robot/Actuators.hpp"
#include "Robotik/Task/Grasp/GraspModel.hpp"

#include "Compages/World/World.hpp"

// ----------------------------------------------------------------------------
TEST(GripperLabel, TheVacuumIsMeasuredTheEntityStaysInTheCell)
{
    compages::world::World world;
    compages::world::EntityId const cube = world.entity("cube").id();
    robotik::VacuumGripper cup("vacuum");
    robotik::HeldObjects held;

    held.hold(cup, cube);
    EXPECT_TRUE(cup.holding());
    EXPECT_EQ(held.of(cup), cube);

    held.drop(cup);
    EXPECT_FALSE(cup.holding());
    EXPECT_FALSE(held.of(cup).valid());

    // An invalid id holds nothing.
    held.hold(cup, compages::world::EntityId{});
    EXPECT_FALSE(cup.holding());
}

// ----------------------------------------------------------------------------
TEST(GripperLabel, LosingTheVacuumForgetsTheLabel)
{
    robotik::VacuumGripper cup("vacuum");
    cup.measureHolding(true);
    cup.label("red_cube");
    EXPECT_EQ(cup.label(), "red_cube");

    // Still held: the label stays.
    cup.measureHolding(true);
    EXPECT_EQ(cup.label(), "red_cube");

    cup.measureHolding(false);
    EXPECT_TRUE(cup.label().empty());
}
