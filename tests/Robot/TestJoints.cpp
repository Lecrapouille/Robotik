// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "main.hpp"

#include "RobotIK/Robot/Backends/RobotBackend.hpp"
#include "RobotIK/Robot/Joints.hpp"
#include "RobotIK/Robot/RobotRuntime.hpp"

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
    robotik::RobotRuntime robot(
        world, assetFile("robotik.robots.simple_revolute_robot"));
    robotik::JointSet& joints = robot.robot().getJoints();
    joints.moveTo(0, 0.5);
    robot.measureJoint(robot.robot().getJoints().requireRevolute(0),
                       robotik::Radians(0.5),
                       robotik::AngularVelocity(1.0));
    EXPECT_FALSE(joints.reached(0, 0.01));
    EXPECT_TRUE(joints.reached(0, 0.01, 2.0));
    robot.measureJoint(robot.robot().getJoints().requireRevolute(0),
                       robotik::Radians(0.5),
                       robotik::AngularVelocity(0.01));
    EXPECT_TRUE(joints.reached(0, 0.01));
}

/// A hardware-like backend: its clock moves by its own steps, not by the dt
/// asked (here two times faster, or not at all).
class OwnClock final: public robotik::RobotBackend
{
public:

    explicit OwnClock(double p_rate) : m_rate(p_rate) {}
    void attach(robotik::RobotRuntime& /*p_robot*/) override {}
    void reset(robotik::RobotRuntime& /*p_robot*/) override
    {
        m_time = 0.0;
    }
    void step(robotik::RobotRuntime& /*p_robot*/,
              robotik::Seconds p_dt) override
    {
        m_time += m_rate * p_dt.value();
    }
    [[nodiscard]] std::optional<robotik::Seconds> time() const override
    {
        return robotik::Seconds(m_time);
    }

private:

    double m_rate;
    double m_time = 0.0;
};

TEST(Clock, ComesFromTheBackend)
{
    compages::world::World world;
    robotik::RobotRuntime robot(
        world, assetFile("robotik.robots.simple_revolute_robot"));
    robot.connect(std::make_unique<OwnClock>(2.0));
    robot.step(robotik::Seconds(0.01));
    robot.step(robotik::Seconds(0.01));
    EXPECT_NEAR(robot.getTime().value(), 0.04, 1e-12);
}

TEST(Clock, CountsTheStepsWithoutABackendClock)
{
    compages::world::World world;
    robotik::RobotRuntime robot(
        world, assetFile("robotik.robots.simple_revolute_robot"));
    robot.step(robotik::Seconds(0.01));
    robot.step(robotik::Seconds(0.01));
    EXPECT_NEAR(robot.getTime().value(), 0.02, 1e-12);
}

// ----------------------------------------------------------------------------
TEST(JointPosture, AnAngleOnAJointThatSlidesIsRefused)
{
    using namespace robotik::literals;
    compages::world::World world;
    robotik::RobotRuntime runtime(
        world, assetFile("robotik.robots.scara.scara_robot"));
    // joint1 turns, end_effector slides: each one in its table.
    runtime.hold({ .angles = { { "joint1", 0.4_rad } },
                   .lengths = { { "end_effector", 0.05_m } } });
    robotik::JointSet const& joints = runtime.robot().getJoints();
    EXPECT_NEAR(joints.position(joints.require("joint1")), 0.4, 1e-12);
    EXPECT_NEAR(joints.position(joints.require("end_effector")), 0.05, 1e-12);

    EXPECT_THROW(runtime.hold({ .angles = { { "end_effector", 0.1_rad } } }),
                 std::invalid_argument);
    EXPECT_THROW(runtime.hold({ .lengths = { { "joint1", 0.1_m } } }),
                 std::invalid_argument);
}

// ----------------------------------------------------------------------------
TEST(JointPosture, TheUrdfSaysWhichJointsTurnAndWhichSlide)
{
    auto const types =
        robotik::readJointTypes(assetFile("robotik.robots.scara.scara_robot"));
    ASSERT_TRUE(types) << types.error();
    EXPECT_EQ(types.value().at("joint1"), robotik::JointType::Revolute);
    EXPECT_EQ(types.value().at("end_effector"), robotik::JointType::Prismatic);
    EXPECT_FALSE(robotik::readJointTypes("no_such.urdf"));
}
