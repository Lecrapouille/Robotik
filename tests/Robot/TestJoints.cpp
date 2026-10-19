// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "main.hpp"

#include "Robotik/Robot/Joints.hpp"

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
