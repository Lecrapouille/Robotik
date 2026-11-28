// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "main.hpp"

#include "RobotIK/Robot/Actuators.hpp"
#include "RobotIK/Robot/StandaloneRobot.hpp"
#include "RobotIK/Task/Grasp/GraspModel.hpp"

#include "Compages/World/World.hpp"

/// A 6-axis arm without physics, with a vacuum cup on its tool frame.
static robotik::StandaloneRobot armWithCup()
{
    robotik::StandaloneRobot robot =
        robotik::require(robotik::StandaloneRobot::open(
            repoFile("data/packs/robotik.robots/robot_6axis.urdf").string(),
            { .physics = std::nullopt }));
    robot.runtime().addActuator<robotik::VacuumGripper>("vacuum");
    return robot;
}

// ----------------------------------------------------------------------------
TEST(GripperLabel, TheVacuumIsMeasuredTheEntityStaysInTheCell)
{
    robotik::StandaloneRobot robot = armWithCup();
    auto& cup =
        *robot.robot().getActuators().find<robotik::VacuumGripper>("vacuum");
    compages::world::EntityId const cube = robot.getWorld().entity("cube").id();
    robotik::HeldObjects held;

    held.hold(robot.runtime(), cup, cube);
    EXPECT_TRUE(cup.holding());
    EXPECT_EQ(held.of(cup), cube);

    held.drop(robot.runtime(), cup);
    EXPECT_FALSE(cup.holding());
    EXPECT_FALSE(held.of(cup).valid());

    // An invalid id holds nothing.
    held.hold(robot.runtime(), cup, compages::world::EntityId{});
    EXPECT_FALSE(cup.holding());
}

// ----------------------------------------------------------------------------
TEST(GripperLabel, LosingTheVacuumForgetsTheLabel)
{
    robotik::StandaloneRobot robot = armWithCup();
    auto& cup =
        *robot.robot().getActuators().find<robotik::VacuumGripper>("vacuum");
    robot.runtime().measureHolding(cup, true);
    cup.label("red_cube");
    EXPECT_EQ(cup.label(), "red_cube");

    // Still held: the label stays.
    robot.runtime().measureHolding(cup, true);
    EXPECT_EQ(cup.label(), "red_cube");

    robot.runtime().measureHolding(cup, false);
    EXPECT_TRUE(cup.label().empty());
}
