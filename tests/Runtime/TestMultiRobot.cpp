// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "main.hpp"

#include "Robotik/Package/Simulation.hpp"
#include "Robotik/Common/Config.hpp"

#include "Compages/World/World.hpp"

#include <cmath>

/// Two copies of the same arm, each with its vacuum cup, face to face.
static char const* const TWO_ARMS = R"(
id: two_arms
entities:
  - id: left
    type: robot
    model: robotik.robots.robot_6axis
    pose: { position: [0.0, -0.6, 0.0] }
    actuators:
      left_arm: { type: joint_group }
  - id: left_cup
    type: tool
    model: robotik.tools.vacuum
    attach: { parent: left.flange, frame: tool_mount }
    actuators:
      left_gripper: { type: vacuum }
  - id: right
    type: robot
    model: robotik.robots.robot_6axis
    pose: { position: [0.0, 0.6, 0.0] }
    actuators:
      right_arm: { type: joint_group }
  - id: right_cup
    type: tool
    model: robotik.tools.vacuum
    attach: { parent: right.flange, frame: tool_mount }
    actuators:
      right_gripper: { type: vacuum }
)";

/// A scenario around the world of @ref TWO_ARMS.
static robotik::Scenario twoArms()
{
    robotik::Scenario scenario;
    scenario.id = "two_arms";
    scenario.world = robotik::readWorld(
        robotik::ConfigNode::parse(TWO_ARMS).take(), ".", testAssets()).take();
    return scenario;
}

TEST(World, SeveralRobotsEachWithItsTool)
{
    robotik::WorldDescription const world = twoArms().world;
    ASSERT_EQ(robotik::robotsOf(world).size(), 2u);
    EXPECT_EQ(robotik::firstRobot(world).id, "left");
    ASSERT_NE(robotik::mountedTool(world, "right"), nullptr);
    EXPECT_EQ(robotik::mountedTool(world, "right")->id, "right_cup");
    EXPECT_EQ(robotik::mountedTool(world)->id, "left_cup");

    // Robots share one resource manager: a device name is used once.
    robotik::ConfigNode bad = robotik::ConfigNode::parse(TWO_ARMS).take();
    EXPECT_TRUE(bad.set("entities.right_cup.actuators",
            robotik::ConfigNode::parse("left_gripper: { type: vacuum }").take()));
    EXPECT_FALSE(robotik::readWorld(bad, ".", testAssets()));
}

TEST(World, ToolTouchesItsWorkpiece)
{
    // The right cup works on the left arm: their contacts are not collisions.
    robotik::ConfigNode node = robotik::ConfigNode::parse(TWO_ARMS).take();
    EXPECT_TRUE(node.set("entities.right_cup.touches",
             robotik::ConfigNode::parse("[left]").take()));
    robotik::WorldDescription const world =
        robotik::readWorld(node, ".", testAssets()).take();
    ASSERT_NE(robotik::findEntity(world, "right_cup"), nullptr);
    EXPECT_EQ(robotik::findEntity(world, "right_cup")->touches,
              std::vector<std::string>{ "left" });

    // Only a tool touches, and only another entity of the world.
    robotik::ConfigNode robot = robotik::ConfigNode::parse(TWO_ARMS).take();
    EXPECT_TRUE(robot.set("entities.right.touches", robotik::ConfigNode::parse("[left]").take()));
    EXPECT_FALSE(robotik::readWorld(robot, ".", testAssets()));
    robotik::ConfigNode unknown = robotik::ConfigNode::parse(TWO_ARMS).take();
    EXPECT_TRUE(unknown.set("entities.right_cup.touches",
                robotik::ConfigNode::parse("[tray]").take()));
    EXPECT_FALSE(robotik::readWorld(unknown, ".", testAssets()));
}

TEST(Simulation, TwoCopiesOfAModelBodyShareThePhysics)
{
    // Two fixed bodies of the same URDF: their MuJoCo names start with
    // their ids, so they do not clash.
    robotik::ConfigNode const node =
        robotik::ConfigNode::parse(std::string(TWO_ARMS) + R"(
  - { id: stand_a, type: body, model: robotik.tools.vacuum, physics: fixed,
      pose: { position: [0.5, -0.2, 0.0] } }
  - { id: stand_b, type: body, model: robotik.tools.vacuum, physics: fixed,
      pose: { position: [0.5, 0.2, 0.0] } }
)").take();
    robotik::Scenario scenario;
    scenario.id = "two_bodies";
    scenario.world = robotik::readWorld(node, ".", testAssets()).take();
    compages::world::World world;
    robotik::Simulation simulation(world, scenario);
    simulation.step(robotik::Seconds(0.01));
    EXPECT_NEAR(simulation.cell().pose("stand_a").position.y, -0.2, 1e-6);
    EXPECT_NEAR(simulation.cell().pose("stand_b").position.y, 0.2, 1e-6);
    EXPECT_DOUBLE_EQ(simulation.cell().query("touching(stand_a, stand_b)").value(),
                     0.0);
}

TEST(Simulation, TwoRobotsShareOnePhysics)
{
    compages::world::World world;
    robotik::Simulation simulation(world, twoArms());
    ASSERT_EQ(simulation.cell().robots().size(), 2u);
    robotik::RobotSession* left = simulation.cell().robot("left");
    robotik::RobotSession* right = simulation.cell().robot("right");
    ASSERT_NE(left, nullptr);
    ASSERT_NE(right, nullptr);
    EXPECT_EQ(&simulation.cell().robot(), left);
    EXPECT_EQ(&simulation.cell().context("right").robot, right);
    EXPECT_EQ(simulation.cell().context("right").team.size(), 2u);

    // Each robot keeps its own joint names and its own skills.
    EXPECT_NE(left->joints().find("joint1"), robotik::NO_JOINT);
    EXPECT_NE(right->joints().find("joint1"), robotik::NO_JOINT);
    EXPECT_NE(simulation.cell().skills().find("left.Home"), robotik::NO_SKILL);
    EXPECT_NE(simulation.cell().skills().find("right.Stop"), robotik::NO_SKILL);
    EXPECT_EQ(simulation.cell().skills().find("Home"), robotik::NO_SKILL);

    // Bases where the world file puts them.
    EXPECT_NEAR(simulation.cell().pose("left").position.y, -0.6, 1e-9);
    EXPECT_NEAR(simulation.cell().pose("right").position.y, 0.6, 1e-9);

    // Moving the right arm leaves the left one where it is.
    robotik::JointId const joint = right->joints().find("joint1");
    double const left_before = left->joints().position(joint);
    right->joints().moveTo(joint, 0.5);
    for (int i = 0; i < 200; ++i)
    {
        simulation.step(robotik::Seconds(0.01));
    }
    EXPECT_NEAR(right->joints().position(joint), 0.5, 0.02);
    EXPECT_NEAR(left->joints().position(joint), left_before, 1e-3);
    EXPECT_NEAR(left->time().value(), right->time().value(), 1e-12);
    EXPECT_DOUBLE_EQ(simulation.cell().query("collision_count(left)").value(), 0.0);
    EXPECT_DOUBLE_EQ(simulation.cell().query("joint_limits_respected(right)").value(),
                     1.0);
    EXPECT_DOUBLE_EQ(simulation.cell().query("holding(right_gripper)").value(), 0.0);
}
