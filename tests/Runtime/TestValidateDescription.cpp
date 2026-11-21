// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

// A description written in C++ follows the rules of a world file: a broken
// rule is a message, never an abort.

#include "main.hpp"

#include "Robotik/Task/OpenRoboticCell.hpp"

#include <stdexcept>

/// A description with the 6-axis arm alone.
static robotik::RoboticCellDescription oneArm()
{
    robotik::EntitySpec arm;
    arm.id = "arm";
    arm.type = robotik::EntitySpec::Type::Robot;
    arm.model = repoFile("data/packs/robotik.robots/robot_6axis.urdf");
    robotik::RoboticCellDescription description;
    description.world.entities = { arm };
    return description;
}

TEST(ValidateDescription, AnEmptyDescriptionIsAMessage)
{
    auto const opened =
        robotik::openRoboticCell(robotik::RoboticCellDescription{});
    ASSERT_FALSE(opened) << "an empty world must not reach the first robot";
    EXPECT_EQ(opened.error(), "world: an entity of type robot is required");

    compages::world::World world;
    EXPECT_THROW(robotik::RoboticCell(world, robotik::RoboticCellDescription{}),
                 std::runtime_error);
}

TEST(ValidateDescription, TheRulesOfTheWorldFileHoldInCpp)
{
    robotik::RoboticCellDescription twice = oneArm();
    twice.world.entities.push_back(twice.world.entities.front());
    robotik::Result<void> const repeated = robotik::validate(twice);
    ASSERT_FALSE(repeated);
    EXPECT_EQ(repeated.error(), "world: entity 'arm' is declared twice");

    robotik::RoboticCellDescription prop = oneArm();
    robotik::EntitySpec box;
    box.id = "box";
    box.type = robotik::EntitySpec::Type::Prop;
    box.shape = robotik::ShapeSpec{};
    box.physics = robotik::EntitySpec::Physics::Free;
    prop.world.entities.push_back(box);
    robotik::Result<void> const physics = robotik::validate(prop);
    ASSERT_FALSE(physics);
    EXPECT_EQ(physics.error(),
              "world, entity 'box': a prop is a shape without physics");

    robotik::RoboticCellDescription hanging = oneArm();
    robotik::EntitySpec cube;
    cube.id = "cube";
    cube.type = robotik::EntitySpec::Type::Body;
    cube.shape = robotik::ShapeSpec{};
    cube.physics = robotik::EntitySpec::Physics::Free;
    cube.attach = robotik::Attachment{ .parent = "table" };
    hanging.world.entities.push_back(cube);
    auto const opened = robotik::openRoboticCell(hanging);
    ASSERT_FALSE(opened);
    EXPECT_EQ(opened.error(),
              "world, entity 'cube': attach.parent 'table' "
              "is not another entity");

    EXPECT_TRUE(robotik::validate(oneArm()));
}
