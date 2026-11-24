// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "main.hpp"

#include "RobotIK/Robot/Joints.hpp"
#include "RobotIK/Robot/RobotSession.hpp"

#include <limits>

/// A single revolute joint with the given speed ratio.
static robotik::JointSet oneJoint(double p_speed_ratio)
{
    robotik::JointSet joints;
    robotik::RevoluteLimits limits;
    limits.velocity = robotik::AngularVelocity(2.0);
    robotik::Revolute const joint = joints.add("joint", limits);
    joints.gains(joint.id).kp = 100.0;
    joints.gains(joint.id).kd = 0.0;
    joints.gains(joint.id).speed_ratio = p_speed_ratio;
    joints.moveTo(joint.id, 1.0);
    return joints;
}

TEST(Joints, PositionLoopSlewsTheReference)
{
    // 2 rad/s x 0.5 x 1 ms: the reference moves 1 mrad toward the target.
    robotik::JointSet joints = oneJoint(0.5);
    joints.control(robotik::Seconds(0.001));
    EXPECT_NEAR(joints.effort(0), 100.0 * 0.001, 1e-9);
}

TEST(Joints, InfiniteSpeedRatioAimsAtTheTargetAtOnce)
{
    robotik::JointSet joints =
        oneJoint(std::numeric_limits<double>::infinity());
    joints.control(robotik::Seconds(0.001));
    EXPECT_NEAR(joints.effort(0), 100.0, 1e-9);
}

TEST(Joints, ReachedNeedsTheJointToBeSlow)
{
    // A joint passing through its target at speed has not arrived.
    compages::world::World world;
    robotik::RobotSession robot(
        world, assetFile("robotik.robots.simple_revolute_robot"));
    robotik::JointSet& joints = robot.joints();
    joints.moveTo(0, 0.5);
    robot.measureJoint(0, 0.5, 1.0);
    EXPECT_FALSE(joints.reached(0, 0.01));
    EXPECT_TRUE(joints.reached(0, 0.01, 2.0));
    robot.measureJoint(0, 0.5, 0.01);
    EXPECT_TRUE(joints.reached(0, 0.01));
}

