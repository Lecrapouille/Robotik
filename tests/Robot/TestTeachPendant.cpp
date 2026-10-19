// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "main.hpp"

#include "Robotik/Backends/MujocoBackend.hpp"
#include "Robotik/Backends/PinocchioBackend.hpp"
#include "Robotik/Robot/Robot.hpp"
#include "Robotik/Robot/TeachPendant.hpp"

#include "Compages/World/World.hpp"

TEST(TeachPendant, QuinticReturnsToTheRecordedJoints)
{
    compages::world::World world;
    robotik::RobotSession robot(
        world, assetFile("robotik.robots.simple_revolute_robot"));
    robotik::TeachPendant pendant;

    std::size_t const home = pendant.record(robot, "home", 1.0);
    EXPECT_EQ(home, 0u);
    ASSERT_TRUE(pendant.jogJoint(robot, 0, 0.4));
    EXPECT_NEAR(robot.joints().target(0), 0.4, 1e-12);

    ASSERT_TRUE(pendant.goTo(robot, 0));
    EXPECT_TRUE(pendant.playing());
    pendant.update(robot, robotik::Seconds(0.5));
    EXPECT_NEAR(robot.joints().target(0), 0.2, 1e-9);
    EXPECT_TRUE(pendant.playing());

    pendant.update(robot, robotik::Seconds(0.5));
    EXPECT_NEAR(robot.joints().target(0), 0.0, 1e-9);
    EXPECT_FALSE(pendant.playing());
}

TEST(TeachPendant, ToolJogSolvesInverseKinematics)
{
    compages::world::World world;
    std::filesystem::path const urdf = assetFile("robotik.robots.robot_6axis");
    robotik::RobotSession robot(world, urdf);
    auto physics = std::make_unique<robotik::MujocoBackend>();
    (void)physics->load(urdf);
    robot.connect(std::move(physics));
    robot.hold({ { "joint1", 0.0 },
                 { "joint2", 0.3 },
                 { "joint3", 1.3 },
                 { "joint4", 0.0 },
                 { "joint5", 1.54 },
                 { "joint6", 0.0 } });

    robotik::Pose const before = robot.framePose(robot.tool());
    robotik::TeachPendant pendant;
    ASSERT_TRUE(pendant.jogTool(
        robot, robotik::Vector3(0.02, 0.0, 0.0), robotik::Vector3::ZERO));
    EXPECT_TRUE(pendant.error().empty());

    robotik::Seconds const step(0.002);
    for (int i = 0; i < 2000; ++i)
    {
        robot.step(step);
    }
    robotik::Pose const after = robot.framePose(robot.tool());
    EXPECT_NEAR(after.position.x, before.position.x + 0.02, 0.01);
    EXPECT_NEAR(after.position.y, before.position.y, 0.01);
    EXPECT_NEAR(after.position.z, before.position.z, 0.01);
}

TEST(TeachPendant, AnyLinkCanBeTheMovedFrame)
{
    compages::world::World world;
    robotik::RobotSession robot(world, assetFile("robotik.robots.robot_6axis"));
    robot.hold({ { "joint1", 0.0 },
                 { "joint2", 0.3 },
                 { "joint3", 1.3 },
                 { "joint4", 0.0 },
                 { "joint5", 1.54 },
                 { "joint6", 0.0 } });
    std::vector<std::string> const links = robot.kinematics().linkFrames();
    ASSERT_NE(std::find(links.begin(), links.end(), "link3"), links.end());

    robotik::TeachPendant pendant;
    EXPECT_EQ(pendant.frame(robot), robot.tool());
    pendant.frame("link3");
    EXPECT_EQ(pendant.frame(robot), "link3");
    pendant.record(robot, "elbow", 1.0);
    robotik::Pose const elbow = robot.framePose("link3");
    EXPECT_NEAR(pendant.waypoints()[0].pose.position.x, elbow.position.x, 1e-9);
    EXPECT_NEAR(pendant.waypoints()[0].pose.position.z, elbow.position.z, 1e-9);

    pendant.frame({});
    EXPECT_EQ(pendant.frame(robot), robot.tool());
}

TEST(TeachPendant, ImpossibleMoveOfALinkFailsWithoutCrash)
{
    compages::world::World world;
    robotik::RobotSession robot(world, assetFile("robotik.robots.robot_6axis"));
    robot.hold({ { "joint1", 0.0 },
                 { "joint2", 0.3 },
                 { "joint3", 1.3 },
                 { "joint4", 0.0 },
                 { "joint5", 1.54 },
                 { "joint6", 0.0 } });
    robotik::TeachPendant pendant;
    pendant.frame("link1");
    // link1 only turns about z: it cannot translate along x.
    EXPECT_FALSE(pendant.jogTool(
        robot, robotik::Vector3(0.01, 0.0, 0.0), robotik::Vector3::ZERO));
    EXPECT_FALSE(pendant.error().empty());
    for (robotik::JointId id = 0; id < robot.joints().size(); ++id)
    {
        EXPECT_TRUE(std::isfinite(robot.joints().target(id)));
    }
    // A rotation about z is possible.
    EXPECT_TRUE(pendant.jogTool(
        robot, robotik::Vector3::ZERO, robotik::Vector3(0.0, 0.0, 0.1)));
}
