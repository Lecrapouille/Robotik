// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "main.hpp"

#include "RobotIK/Robot/StandaloneRobot.hpp"
#include "RobotIK/Package/StandaloneSimulation.hpp"
#include "RobotIK/Task/StandaloneCell.hpp"

#include <type_traits>

// The world must outlive the robot: it moves, it is never assigned (the
// old world would go before the old robot), and nothing outside reaches it.
static_assert(std::is_move_constructible_v<robotik::StandaloneRobot>);
static_assert(!std::is_move_assignable_v<robotik::StandaloneRobot>);
static_assert(!std::is_copy_constructible_v<robotik::StandaloneRobot>);
static_assert(!std::is_default_constructible_v<robotik::StandaloneRobot>);
static_assert(std::is_move_constructible_v<robotik::StandaloneCell>);
static_assert(!std::is_move_assignable_v<robotik::StandaloneCell>);
static_assert(!std::is_default_constructible_v<robotik::StandaloneCell>);
static_assert(std::is_move_constructible_v<robotik::StandaloneSimulation>);
static_assert(!std::is_move_assignable_v<robotik::StandaloneSimulation>);
static_assert(!std::is_default_constructible_v<robotik::StandaloneSimulation>);

// ----------------------------------------------------------------------------
TEST(StandaloneRobot, MovesAJointInAFewLines)
{
    auto arm = robotik::StandaloneRobot::open(
        repoFile("data/packs/robotik.robots/robot_6axis.urdf").string());
    ASSERT_TRUE(arm) << arm.error();
    robotik::JointSet& joints = arm.value().robot().getJoints();
    robotik::Revolute const joint = joints.requireRevolute("joint2");
    joints.moveTo(joint, robotik::Radians(0.3));
    for (int i = 0; i < 400; ++i)
    {
        arm.value().runtime().step(robotik::Seconds(0.01));
    }
    EXPECT_NEAR(joints.position(joint).value(), 0.3, 0.02);
}

// ----------------------------------------------------------------------------
TEST(StandaloneRobot, MissingModelIsAMessageNotAnException)
{
    auto missing =
        robotik::StandaloneRobot::open("robotik.robots.no_such_robot");
    ASSERT_FALSE(missing);
    EXPECT_NE(missing.error().find("no_such_robot"), std::string::npos);
}

// ----------------------------------------------------------------------------
TEST(StandaloneRobot, ResolvedLinkGivesTheSamePoseAsItsName)
{
    auto arm = robotik::StandaloneRobot::open(
        repoFile("data/packs/robotik.robots/robot_6axis.urdf").string(),
        { .physics = std::nullopt });
    ASSERT_TRUE(arm) << arm.error();
    robotik::JointSet& joints = arm.value().robot().getJoints();
    joints.moveTo(joints.requireRevolute("joint2"), robotik::Radians(0.5));
    arm.value().runtime().step(robotik::Seconds(0.01));

    robotik::LinkRef const wrist = arm.value().robot().requireLink("link6");
    robotik::Pose const by_name =
        arm.value().robot().requireWorldPose(std::string("link6"));
    robotik::Pose const by_ref = arm.value().robot().getWorldPose(wrist);
    EXPECT_DOUBLE_EQ(by_ref.position.x, by_name.position.x);
    EXPECT_DOUBLE_EQ(by_ref.position.z, by_name.position.z);
    robotik::Pose const base =
        arm.value().robot().getWorldPose(arm.value().robot().requireLink(""));
    EXPECT_DOUBLE_EQ(base.position.z,
                     arm.value().robot().getBase().pose.position.z);
    EXPECT_THROW((void)arm.value().robot().requireLink("no_such_link"),
                 std::invalid_argument);
}

// ----------------------------------------------------------------------------
TEST(StandaloneRobot, LinkOfAnotherRobotIsAMistakeInTheCode)
{
    std::string const urdf =
        repoFile("data/packs/robotik.robots/robot_6axis.urdf").string();
    auto left =
        robotik::StandaloneRobot::open(urdf, { .physics = std::nullopt });
    auto right =
        robotik::StandaloneRobot::open(urdf, { .physics = std::nullopt });
    ASSERT_TRUE(left) << left.error();
    ASSERT_TRUE(right) << right.error();

    // Same URDF, same frame index: still refused, it is not its link.
    robotik::LinkRef const wrist = left.value().robot().requireLink("link6");
    EXPECT_THROW((void)right.value().robot().getWorldPose(wrist),
                 std::logic_error);
    EXPECT_THROW((void)left.value().robot().getWorldPose(robotik::LinkRef{}),
                 std::logic_error);

    // The model is built once: the handle stays good after a reset.
    left.value().runtime().reset();
    EXPECT_NO_THROW((void)left.value().robot().getWorldPose(wrist));
}

// ----------------------------------------------------------------------------
TEST(StandaloneRobot, SkillsReadTheEstimatedBaseSensorsTheTrueOne)
{
    auto arm = robotik::StandaloneRobot::open(
        repoFile("data/packs/robotik.robots/robot_6axis.urdf").string(),
        { .physics = std::nullopt });
    ASSERT_TRUE(arm) << arm.error();
    robotik::RobotRuntime& robot = arm.value().runtime();
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

// ----------------------------------------------------------------------------
TEST(StandaloneRobot, AToolIsMountedTheSameWayInEveryPart)
{
    using namespace robotik::literals;
    // The kinematics, the drawn world and MuJoCo mount the gripper through
    // the same ToolMount: its finger joints join the robot, its tcp is the
    // work frame, and the physics steps with it.
    auto arm = robotik::StandaloneRobot::open(
        "robotik.robots.robot_6axis",
        { .tool = { .urdf = assetFile("robotik.tools.gripper") } });
    ASSERT_TRUE(arm) << arm.error();
    robotik::Robot const& robot = arm.value().robot();
    EXPECT_EQ(robot.getJoints().size(), 8U);
    EXPECT_EQ(robot.getTool(), "tcp");
    EXPECT_NE(arm.value().runtime().getPhysics(), nullptr);
    arm.value().runtime().step(0.01_s);

    // A flange that the robot does not have is a message, not a crash.
    auto wrong = robotik::StandaloneRobot::open(
        "robotik.robots.robot_6axis",
        { .tool = { .urdf = assetFile("robotik.tools.gripper"),
                    .flange = "no_such_link" } });
    ASSERT_FALSE(wrong);
    EXPECT_NE(wrong.error().find("no_such_link"), std::string::npos);
}

// ----------------------------------------------------------------------------
TEST(StandaloneCell, TheToolMountComesFromTheAttachBlock)
{
    robotik::WorldDescription world;
    robotik::EntitySpec arm;
    arm.id = "arm";
    arm.type = robotik::EntitySpec::Type::Robot;
    robotik::EntitySpec drill;
    drill.id = "drill";
    drill.type = robotik::EntitySpec::Type::Tool;
    drill.model = "drill.urdf";
    drill.attach = robotik::Attachment{ "arm", "link6", "", {} };
    world.entities = { arm, drill };

    robotik::ToolMount const mount = robotik::toolMountOf(world, "arm");
    EXPECT_EQ(mount.urdf, "drill.urdf");
    EXPECT_EQ(mount.flange, "link6");     // from attach.parent: arm.link6
    EXPECT_EQ(mount.mount, "tool_mount"); // no attach.frame: the default
    EXPECT_FALSE(robotik::hasTool(robotik::toolMountOf(world, "nobody")));
}

// ----------------------------------------------------------------------------
TEST(StandaloneSimulation, AMissingScenarioFileIsAMessage)
{
    auto const opened =
        robotik::StandaloneSimulation::open(std::filesystem::path(
            "no_such_dir/no_such.scenario.yaml"));
    ASSERT_FALSE(opened);
    EXPECT_NE(opened.error().find("no_such.scenario.yaml"), std::string::npos)
        << opened.error();
}
