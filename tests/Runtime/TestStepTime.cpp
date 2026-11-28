// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

// A step moves the clock forward, and the physics covers the same time.

#include "main.hpp"

#include "RobotIK/Task/StandaloneCell.hpp"

#include <limits>
#include <stdexcept>

/// The 6-axis arm and a free cube 1 m above the floor, rigid physics.
static robotik::RoboticCellDescription fallingCube()
{
    robotik::EntitySpec arm;
    arm.id = "arm";
    arm.type = robotik::EntitySpec::Type::Robot;
    arm.model = repoFile("data/packs/robotik.robots/robot_6axis.urdf");
    robotik::EntitySpec cube;
    cube.id = "cube";
    cube.type = robotik::EntitySpec::Type::Body;
    cube.shape = robotik::ShapeSpec{};
    cube.physics = robotik::EntitySpec::Physics::Free;
    cube.pose.position = { 1.0, 0.0, 1.0 };
    robotik::RoboticCellDescription description;
    description.world.floor = true;
    description.world.entities = { arm, cube };
    description.physics.fidelity = robotik::PhysicsFidelity::Rigid;
    return description;
}

TEST(StepTime, AStepThatDoesNotMoveTheClockIsRefused)
{
    auto opened = robotik::StandaloneCell::open(fallingCube());
    ASSERT_TRUE(opened) << opened.error();
    robotik::RoboticCell& cell = opened.value().cell();
    double const nan = std::numeric_limits<double>::quiet_NaN();
    EXPECT_THROW(cell.step(robotik::Seconds(0.0)), std::invalid_argument);
    EXPECT_THROW(cell.step(robotik::Seconds(-0.01)), std::invalid_argument);
    EXPECT_THROW(cell.step(robotik::Seconds(nan)), std::invalid_argument);
    EXPECT_THROW((void)cell.run(robotik::Seconds(1.0), robotik::Seconds(0.0)),
                 std::invalid_argument);
    EXPECT_THROW((void)cell.run(robotik::Seconds(-1.0)), std::invalid_argument);
    EXPECT_EQ(cell.time(), robotik::Seconds(0.0));
}

TEST(StepTime, ThePhysicsCoversTheTimeOfTheStep)
{
    // 0.0025 s is not a multiple of the MuJoCo timestep (0.002 s): before,
    // each call made one physics step, so the cube fell for 0.08 s while
    // the clock said 0.1 s.
    double heights[2] = {};
    double const steps[2] = { 0.01, 0.0025 };
    for (int i = 0; i < 2; ++i)
    {
        auto opened = robotik::StandaloneCell::open(fallingCube());
        ASSERT_TRUE(opened) << opened.error();
        robotik::RoboticCell& cell = opened.value().cell();
        while (cell.time().value() < 0.1 - 1e-9)
        {
            cell.step(robotik::Seconds(steps[i]));
        }
        EXPECT_NEAR(cell.time().value(), 0.1, 1e-9);
        heights[i] = cell.requirePose("cube").position.z;
    }
    EXPECT_LT(heights[0], 1.0 - 0.04) << "the cube falls about 5 cm in 0.1 s";
    EXPECT_NEAR(heights[0], heights[1], 1e-3);
}
