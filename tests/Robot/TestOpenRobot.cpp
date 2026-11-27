// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "main.hpp"

#include "RobotIK/Robot/OpenRobot.hpp"

// ----------------------------------------------------------------------------
TEST(OpenRobot, MovesAJointInAFewLines)
{
    auto arm = robotik::openRobot(
        repoFile("data/packs/robotik.robots/robot_6axis.urdf").string());
    ASSERT_TRUE(arm) << arm.error();
    robotik::JointSet& joints = arm.value()->robot().getJoints();
    robotik::Revolute const joint = joints.requireRevolute("joint2");
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
    auto arm = robotik::openRobot(
        repoFile("data/packs/robotik.robots/robot_6axis.urdf").string(),
        options);
    ASSERT_TRUE(arm) << arm.error();
    robotik::JointSet& joints = arm.value()->robot().getJoints();
    joints.moveTo(joints.requireRevolute("joint2"), robotik::Radians(0.5));
    arm.value()->step(robotik::Seconds(0.01));

    robotik::LinkRef const wrist = arm.value()->robot().requireLink("link6");
    robotik::Pose const by_name =
        arm.value()->robot().requireWorldPose(std::string("link6"));
    robotik::Pose const by_ref = arm.value()->robot().getWorldPose(wrist);
    EXPECT_DOUBLE_EQ(by_ref.position.x, by_name.position.x);
    EXPECT_DOUBLE_EQ(by_ref.position.z, by_name.position.z);
    robotik::Pose const base =
        arm.value()->robot().getWorldPose(arm.value()->robot().requireLink(""));
    EXPECT_DOUBLE_EQ(base.position.z,
                     arm.value()->robot().getBase().pose.position.z);
    EXPECT_THROW((void)arm.value()->robot().requireLink("no_such_link"),
                 std::invalid_argument);
}

// ----------------------------------------------------------------------------
TEST(OpenRobot, LinkOfAnotherRobotIsAMistakeInTheCode)
{
    robotik::RobotOptions options;
    options.physics = false;
    std::string const urdf =
        repoFile("data/packs/robotik.robots/robot_6axis.urdf").string();
    auto left = robotik::openRobot(urdf, options);
    auto right = robotik::openRobot(urdf, options);
    ASSERT_TRUE(left) << left.error();
    ASSERT_TRUE(right) << right.error();

    // Same URDF, same frame index: still refused, it is not its link.
    robotik::LinkRef const wrist = left.value()->robot().requireLink("link6");
    EXPECT_THROW((void)right.value()->robot().getWorldPose(wrist),
                 std::logic_error);
    EXPECT_THROW((void)left.value()->robot().getWorldPose(robotik::LinkRef{}),
                 std::logic_error);

    // The model is built once: the handle stays good after a reset.
    left.value()->reset();
    EXPECT_NO_THROW((void)left.value()->robot().getWorldPose(wrist));
}

// ----------------------------------------------------------------------------
TEST(OpenRobot, SkillsReadTheEstimatedBaseSensorsTheTrueOne)
{
    robotik::RobotOptions options;
    options.physics = false;
    auto arm = robotik::openRobot(
        repoFile("data/packs/robotik.robots/robot_6axis.urdf").string(),
        options);
    ASSERT_TRUE(arm) << arm.error();
    robotik::RobotRuntime& robot = *arm.value();
    robotik::LinkRef const wrist = robot.robot().requireLink("link6");
    robotik::BaseState moved;
    moved.pose.position = robotik::Vector3(1.0, 0.0, 0.0);

    // The mock: the estimate follows the truth.
    EXPECT_EQ(robot.getLocalization(), robotik::LocalizationFidelity::Truth);
    robot.measureBase(moved);
    EXPECT_DOUBLE_EQ(robot.robot().getBase().pose.position.x, 1.0);

    // A localization of the plugin: the truth moves, the estimate waits.
    robot.setLocalization(robotik::LocalizationFidelity::Plugin);
    moved.pose.position.x = 2.0;
    robot.measureBase(moved);
    EXPECT_DOUBLE_EQ(robot.getTrueBase().pose.position.x, 2.0);
    EXPECT_DOUBLE_EQ(robot.robot().getBase().pose.position.x, 1.0);

    robotik::BaseState estimate = moved;
    estimate.pose.position.x = 1.9;
    robot.estimateBase(estimate);
    EXPECT_NEAR(robot.getTrueWorldPose(wrist).position.x -
                    robot.robot().getWorldPose(wrist).position.x,
                0.1,
                1e-12);

    // A reset puts both at the start pose.
    robot.reset();
    EXPECT_DOUBLE_EQ(robot.robot().getBase().pose.position.x, 0.0);
    EXPECT_DOUBLE_EQ(robot.getTrueBase().pose.position.x, 0.0);
}
