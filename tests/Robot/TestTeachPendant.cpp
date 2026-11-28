// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "main.hpp"

#include "RobotIK/Robot/Backends/MujocoBackend.hpp"
#include "RobotIK/Robot/Kinematics.hpp"
#include "RobotIK/Robot/RobotRuntime.hpp"
#include "RobotIK/Robot/StandaloneRobot.hpp"
#include "RobotIK/Robot/TeachPendant.hpp"

#include "Compages/World/World.hpp"

#include <algorithm>
#include <array>
#include <optional>
#include <vector>

using namespace robotik::literals;

/// The 6-axis arm held at a ready posture, with MuJoCo or without physics.
static robotik::StandaloneRobot readyArm(bool p_physics)
{
    robotik::StandaloneRobot::Setup setup;
    setup.home.angles = { { "joint1", 0.0_rad },  { "joint2", 0.3_rad },
                          { "joint3", 1.3_rad },  { "joint4", 0.0_rad },
                          { "joint5", 1.54_rad }, { "joint6", 0.0_rad } };
    if (!p_physics)
    {
        setup.physics = std::nullopt;
    }
    return robotik::require(
        robotik::StandaloneRobot::open("robotik.robots.robot_6axis", setup));
}

TEST(TeachPendant, QuinticReturnsToTheRecordedJoints)
{
    compages::world::World world;
    robotik::RobotRuntime runtime(
        world, assetFile("robotik.robots.simple_revolute_robot"));
    robotik::Robot& robot = runtime.robot();
    robotik::TeachPendant pendant;

    std::size_t const home = pendant.record(robot, "home", 1.0_s);
    EXPECT_EQ(home, 0U);
    ASSERT_TRUE(
        pendant.jogJoint(robot, robot.getJoints().requireRevolute(0), 0.4_rad));
    EXPECT_NEAR(robot.getJoints().target(0), 0.4, 1e-12);

    ASSERT_TRUE(pendant.goTo(robot, 0));
    EXPECT_TRUE(pendant.isPlaying());
    pendant.update(robot, robotik::Seconds(0.5));
    EXPECT_NEAR(robot.getJoints().target(0), 0.2, 1e-9);
    EXPECT_TRUE(pendant.isPlaying());

    pendant.update(robot, robotik::Seconds(0.5));
    EXPECT_NEAR(robot.getJoints().target(0), 0.0, 1e-9);
    EXPECT_FALSE(pendant.isPlaying());
}

TEST(TeachPendant, ToolJogSolvesInverseKinematics)
{
    robotik::StandaloneRobot arm = readyArm(true);
    robotik::RobotRuntime& runtime = arm.runtime();
    robotik::Robot& robot = arm.robot();

    robotik::Pose const before = robot.requireFramePose(robot.getTool());
    robotik::TeachPendant pendant;
    ASSERT_TRUE(pendant.jogTool(
        robot, { 0.02_m, 0.0_m, 0.0_m }, robotik::Vector3rad::ZERO));
    EXPECT_TRUE(pendant.getError().empty());

    robotik::Seconds const step(0.002);
    for (int i = 0; i < 2000; ++i)
    {
        runtime.step(step);
    }
    robotik::Pose const after = robot.requireFramePose(robot.getTool());
    EXPECT_NEAR(after.position.x, before.position.x + 0.02, 0.01);
    EXPECT_NEAR(after.position.y, before.position.y, 0.01);
    EXPECT_NEAR(after.position.z, before.position.z, 0.01);
}

TEST(TeachPendant, AnyLinkCanBeTheMovedFrame)
{
    robotik::StandaloneRobot arm = readyArm(false);
    robotik::Robot& robot = arm.robot();
    std::vector<std::string> const links = robot.getKinematics().linkFrames();
    ASSERT_NE(std::ranges::find(links, "link3"), links.end());

    robotik::TeachPendant pendant;
    EXPECT_EQ(pendant.getFrame(robot), robot.getTool());
    pendant.setFrame("link3");
    EXPECT_EQ(pendant.getFrame(robot), "link3");
    pendant.record(robot, "elbow", 1.0_s);
    robotik::Pose const elbow = robot.requireFramePose("link3");
    EXPECT_NEAR(
        pendant.getWaypoints()[0].pose.position.x, elbow.position.x, 1e-9);
    EXPECT_NEAR(
        pendant.getWaypoints()[0].pose.position.z, elbow.position.z, 1e-9);

    pendant.setFrame({});
    EXPECT_EQ(pendant.getFrame(robot), robot.getTool());
}

TEST(TeachPendant, ImpossibleMoveOfALinkFailsWithoutCrash)
{
    robotik::StandaloneRobot arm = readyArm(false);
    robotik::Robot& robot = arm.robot();
    robotik::TeachPendant pendant;
    pendant.setFrame("link1");
    // link1 only turns about z: it cannot translate along x.
    EXPECT_FALSE(pendant.jogTool(
        robot, { 0.01_m, 0.0_m, 0.0_m }, robotik::Vector3rad::ZERO));
    EXPECT_FALSE(pendant.getError().empty());
    for (robotik::JointId id = 0; id < robot.getJoints().size(); ++id)
    {
        EXPECT_TRUE(std::isfinite(robot.getJoints().target(id)));
    }
    // A rotation about z is possible.
    EXPECT_TRUE(pendant.jogTool(
        robot, robotik::Vector3m::ZERO, { 0.0_rad, 0.0_rad, 0.1_rad }));
}

TEST(TeachPendant, AxesFollowTheJointsAboveTheFrame)
{
    robotik::StandaloneRobot arm = readyArm(false);
    robotik::Robot& robot = arm.robot();
    robotik::Kinematics const& kinematics = robot.getKinematics();
    EXPECT_EQ(kinematics.requireChainDofs("link1"), 1U);
    EXPECT_EQ(kinematics.requireChainDofs("link3"), 3U);
    EXPECT_GE(kinematics.requireChainDofs(robot.getTool()), 6U);

    robotik::TeachPendant pendant;
    std::array<bool, 6> const tool = pendant.getJogAxes(robot);
    EXPECT_EQ(std::ranges::count(tool, true), 6);

    // link1 turns about z only.
    pendant.setFrame("link1");
    std::array<bool, 6> const shoulder = pendant.getJogAxes(robot);
    EXPECT_EQ(shoulder,
              (std::array<bool, 6>{ false, false, false, false, false, true }));
    EXPECT_EQ(pendant.getJogGoal(robot, true), robotik::IkGoal::Orientation);
}

TEST(TeachPendant, ALinkWithFewJointsMovesItsPositionAlone)
{
    robotik::StandaloneRobot arm = readyArm(false);
    robotik::Robot& robot = arm.robot();
    robotik::TeachPendant pendant;
    pendant.setFrame("link3");
    // joint3 turns about the origin of link3: two joints move it, on a
    // sphere about the shoulder. X is mostly reachable, Z is not.
    std::array<bool, 6> const elbow = pendant.getJogAxes(robot);
    EXPECT_TRUE(elbow[0]);
    EXPECT_FALSE(elbow[2]);
    EXPECT_EQ(pendant.getJogGoal(robot, false), robotik::IkGoal::Position);

    robotik::Pose const before = robot.requireFramePose("link3");
    robotik::Pose target = before;
    target.position.x -= 0.02;
    std::optional<std::vector<double>> const solution =
        robot.inverseKinematics("link3", target, robotik::IkGoal::Position);
    ASSERT_TRUE(solution.has_value());
    EXPECT_TRUE(pendant.jogTool(
        robot, { -0.02_m, 0.0_m, 0.0_m }, robotik::Vector3rad::ZERO));
    // Out of the sphere: the closest point, not a failure.
    EXPECT_TRUE(pendant.jogTool(
        robot, { 0.02_m, 0.0_m, 0.0_m }, robotik::Vector3rad::ZERO));
    EXPECT_FALSE(pendant.jogTool(
        robot, { 0.0_m, 0.0_m, 0.02_m }, robotik::Vector3rad::ZERO));
    EXPECT_FALSE(pendant.getError().empty());
}

// The example of the TeachPendant class comment.
TEST(TeachPendant, HeaderExampleTeachesAndReplaysTwoPositions)
{
    robotik::StandaloneRobot arm = readyArm(true);
    robotik::RobotRuntime& runtime = arm.runtime();
    robotik::Robot& robot = arm.robot();
    robotik::Revolute const wrist = robot.getJoints().requireRevolute("joint6");
    robotik::Seconds const dt(0.002);

    robotik::TeachPendant pendant;
    ASSERT_TRUE(pendant.jogTool(
        robot, { 0.0_m, 0.0_m, 0.01_m }, robotik::Vector3rad::ZERO));
    for (int i = 0; i < 1000; ++i)
    {
        runtime.step(dt);
    }
    pendant.record(robot, "above", 2.0_s);
    ASSERT_TRUE(pendant.jogJoint(robot, wrist, 0.2_rad));
    for (int i = 0; i < 1000; ++i)
    {
        runtime.step(dt);
    }
    pendant.record(robot, "turned", 1.0_s);
    ASSERT_EQ(pendant.getWaypoints().size(), 2U);

    ASSERT_TRUE(pendant.play(robot, true));
    EXPECT_EQ(pendant.getTargetIndex(), 0);
    for (int i = 0; i < 1600 && pendant.isPlaying(); ++i) // 2 s + 1 s + a bit
    {
        pendant.update(robot, dt);
        runtime.step(dt);
    }
    EXPECT_TRUE(pendant.isPlaying()); // the loop never ends by itself
    pendant.stop(robot);
    EXPECT_FALSE(pendant.isPlaying());
    EXPECT_EQ(pendant.getTargetIndex(), -1);
}

TEST(TeachPendant, AWaypointOfOtherJointsIsRefusedWithTheSameCount)
{
    // Two arms with six joints each, but named joint1.. and joint_1..: the
    // angles of one mean nothing on the other.
    compages::world::World world;
    robotik::RobotRuntime first(world, assetFile("robotik.robots.robot_6axis"));
    robotik::RobotRuntime second(world,
                                 assetFile("robotik.robots.abb.irb2400"));
    ASSERT_EQ(first.robot().getJoints().size(),
              second.robot().getJoints().size());
    robotik::TeachPendant pendant;
    (void)pendant.record(first.robot(), "home", 1.0_s);
    EXPECT_EQ(pendant.getWaypoints()[0].joint_names,
              first.robot().getModel().joints);

    EXPECT_TRUE(pendant.goTo(first.robot(), 0));
    pendant.stop(first.robot());
    EXPECT_FALSE(pendant.goTo(second.robot(), 0));
    EXPECT_FALSE(pendant.isPlaying());
}
