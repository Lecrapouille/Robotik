// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

// The truth is where things are now, not where the file put them.

#include "main.hpp"

#include "Robotik/Task/OpenRoboticCell.hpp"

TEST(TruthPose, APropTurnedInTheWorldReportsItsRotation)
{
    robotik::EntitySpec arm;
    arm.id = "arm";
    arm.type = robotik::EntitySpec::Type::Robot;
    arm.model = repoFile("data/packs/robotik.robots/robot_6axis.urdf");
    robotik::EntitySpec box;
    box.id = "box";
    box.type = robotik::EntitySpec::Type::Prop;
    box.shape = robotik::ShapeSpec{};
    box.pose.position = { 0.5, 0.0, 0.0 };
    robotik::RoboticCellDescription description;
    description.world.entities = { arm, box };

    auto opened = robotik::openRoboticCell(std::move(description));
    ASSERT_TRUE(opened) << opened.error();
    robotik::RoboticCell& cell = *opened.value().cell;
    EXPECT_NEAR(
        robotik::yaw(cell.requirePose("box").rotation).value(), 0.0, 1e-6);

    // A quarter turn about z, as a push or a grasp would do.
    cell.entity("box").rotation(
        compages::Quatf(0.70710678F, 0.0F, 0.0F, 0.70710678F));
    EXPECT_NEAR(robotik::yaw(cell.requirePose("box").rotation).value(),
                1.5707963,
                1e-5);
}
