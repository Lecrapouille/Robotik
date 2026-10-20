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

// ----------------------------------------------------------------------------
TEST(OpenRobot, ResolvedLinkGivesTheSamePoseAsItsName)
{
    robotik::RobotOptions options;
    options.physics = false;
    auto arm = robotik::openRobot(repoFile("data/packs/robotik.robots/robot_6axis.urdf").string(),
                                  options);
    ASSERT_TRUE(arm) << arm.error();
    robotik::JointSet& joints = arm.value()->joints();
    joints.moveTo(joints.revolute("joint2"), robotik::Radians(0.5));
    arm.value()->step(robotik::Seconds(0.01));

    robotik::LinkRef const wrist = arm.value()->resolve("link6");
    EXPECT_TRUE(wrist.entity);
    robotik::Pose const by_name = arm.value()->worldPose(std::string("link6"));
    robotik::Pose const by_ref = arm.value()->worldPose(wrist);
    EXPECT_DOUBLE_EQ(by_ref.position.x, by_name.position.x);
    EXPECT_DOUBLE_EQ(by_ref.position.z, by_name.position.z);
    EXPECT_EQ(arm.value()->resolve("").frame, robotik::LinkRef::BASE);
    EXPECT_THROW((void)arm.value()->resolve("no_such_link"), std::invalid_argument);
}
