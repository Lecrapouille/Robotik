// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "main.hpp"

#include "Robotik/Robot/OpenRobot.hpp"

// ----------------------------------------------------------------------------
TEST(OpenRobot, MovesAJointInAFewLines)
{
    auto arm = robotik::openRobot(repoFile("data/packs/robotik.robots/robot_6axis.urdf").string());
    ASSERT_TRUE(arm) << arm.error();
    robotik::JointSet& joints = arm.value()->joints();
    robotik::Revolute const joint = joints.revolute("joint2");
    joints.moveTo(joint, robotik::Radians(0.3));
    for (int i = 0; i < 400; ++i)
    {
        arm.value()->step(robotik::Seconds(0.01));
    }
    EXPECT_NEAR(joints.position(joint).value(), 0.3, 0.02);
}

// ----------------------------------------------------------------------------
TEST(OpenRobot, MissingModelIsAMessageNotAnException)
{
    auto missing = robotik::openRobot("robotik.robots.no_such_robot");
    ASSERT_FALSE(missing);
    EXPECT_NE(missing.error().find("no_such_robot"), std::string::npos);
}
