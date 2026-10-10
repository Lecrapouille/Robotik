// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "main.hpp"

#include "Robotik/Backends/MujocoBackend.hpp"
#include "Robotik/Robot/Robot.hpp"
#include "Robotik/Robot/TeachPendant.hpp"

#include "Compages/World/World.hpp"

TEST(TeachPendant, QuinticReturnsToTheRecordedJoints)
{
    compages::world::World world;
    robotik::RobotSession robot(world, dataFile("simple_revolute_robot.urdf"));
    robotik::TeachPendant pendant;

    std::size_t const home = pendant.record(robot, "home", 1.0);
    EXPECT_EQ(home, 0u);
    ASSERT_TRUE(pendant.jogJoint(robot, 0, 0.4));
    EXPECT_NEAR(robot.joints().target(0), 0.4, 1e-12);

    ASSERT_TRUE(pendant.goTo(robot, 0));
    EXPECT_TRUE(pendant.playing());
    pendant.update(robot, Seconds(0.5));
    EXPECT_NEAR(robot.joints().target(0), 0.2, 1e-9);
    EXPECT_TRUE(pendant.playing());

    pendant.update(robot, Seconds(0.5));
    EXPECT_NEAR(robot.joints().target(0), 0.0, 1e-9);
    EXPECT_FALSE(pendant.playing());
}

TEST(TeachPendant, ToolJogSolvesInverseKinematics)
{
    compages::world::World world;
    std::filesystem::path const urdf = dataFile("robot_6axis.urdf");
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
        robot, robotik::Vector3(0.02, 0.0, 0.0), robotik::zero3()));
    EXPECT_TRUE(pendant.error().empty());

    Seconds const step(0.002);
    for (int i = 0; i < 2000; ++i)
    {
        robot.step(step);
    }
    robotik::Pose const after = robot.framePose(robot.tool());
    EXPECT_NEAR(after.position.x, before.position.x + 0.02, 0.01);
    EXPECT_NEAR(after.position.y, before.position.y, 0.01);
    EXPECT_NEAR(after.position.z, before.position.z, 0.01);
}
