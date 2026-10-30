// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

// The conveyor belt, a device of the cell: its skill, the parts it carries
// with and without physics, a failed belt.

#include "main.hpp"

#include "Robotik/Common/Config.hpp"
#include "Robotik/Package/Scenario/World.hpp"
#include "Robotik/Task/Cell.hpp"

#include "Compages/World/World.hpp"

/// Speed of the belt (m/s).
static constexpr double SPEED = 0.2;

/// An arm away from a belt of 1 m along +x; a cube resting on the belt.
static robotik::CellDescription beltCell(robotik::Fidelity p_fidelity)
{
    robotik::CellDescription cell;
    robotik::EntitySpec arm;
    arm.id = "arm";
    arm.type = robotik::EntitySpec::Type::Robot;
    arm.model = repoFile("data/packs/robotik.robots/robot_6axis.urdf");

    robotik::EntitySpec belt;
    belt.id = "feeder";
    belt.type = robotik::EntitySpec::Type::Body;
    belt.physics = robotik::EntitySpec::Physics::Fixed;
    belt.shape = robotik::ShapeSpec{ .size = robotik::Vector3(1.0, 0.2, 0.1) };
    belt.pose.position = robotik::Vector3(0.5, -0.8, 0.05);
    robotik::ActuatorSpec drive;
    drive.name = "feeder_belt";
    drive.type = robotik::ActuatorSpec::Type::Conveyor;
    drive.speed = robotik::LinearVelocity(SPEED);
    belt.actuators = { drive };

    robotik::EntitySpec cube;
    cube.id = "cube";
    cube.shape = robotik::ShapeSpec{};
    cube.pose.position = robotik::Vector3(0.2, -0.8, 0.12);
    cell.world.entities = { arm, belt, cube };
    cell.physics.fidelity = p_fidelity;
    return cell;
}

/// Steps the cell during @p_duration.
static void run(robotik::Cell& p_cell, double p_duration)
{
    robotik::Seconds const end = p_cell.time() + robotik::Seconds(p_duration);
    while (p_cell.time() < end)
    {
        p_cell.step(robotik::Seconds(0.01));
    }
}

TEST(ConveyorBelt, RunsWhileItsSkillRuns)
{
    compages::world::World world;
    robotik::Cell cell(world, beltCell(robotik::Fidelity::Abstract));
    auto* belt = cell.devices().find<robotik::ConveyorBelt>("feeder_belt");
    ASSERT_NE(belt, nullptr);
    EXPECT_EQ(belt->entity(), "feeder");
    robotik::SkillId const skill = cell.skills().find("feeder_belt.Run");
    ASSERT_NE(skill, robotik::NO_SKILL);

    // Stopped: the cube stays.
    run(cell, 0.5);
    EXPECT_NEAR(cell.pose("cube").position.x, 0.2, 1e-6);

    // One second at 0.2 m/s along +x.
    cell.skills().request(skill);
    run(cell, 1.0);
    EXPECT_TRUE(belt->running());
    EXPECT_NEAR(cell.pose("cube").position.x, 0.4, 0.01);
    EXPECT_NEAR(cell.pose("cube").position.y, -0.8, 1e-6);

    // Cancelled: the belt stops, the cube too.
    cell.skills().cancel(skill);
    run(cell, 0.5);
    EXPECT_FALSE(belt->running());
    EXPECT_NEAR(cell.pose("cube").position.x, 0.4, 0.01);
}

TEST(ConveyorBelt, StopsTheCubeAtItsEnd)
{
    compages::world::World world;
    robotik::Cell cell(world, beltCell(robotik::Fidelity::Abstract));
    cell.devices().find<robotik::ConveyorBelt>("feeder_belt")->run(true);
    run(cell, 5.0);
    // Without physics the part stops when its centre leaves the belt.
    EXPECT_NEAR(cell.pose("cube").position.x, 1.0, 0.01);
}

TEST(ConveyorBelt, PhysicsCarriesARigidCube)
{
    compages::world::World world;
    robotik::Cell cell(world, beltCell(robotik::Fidelity::Rigid));
    // The cube settles on the belt.
    run(cell, 0.3);
    double const start = cell.pose("cube").position.x;
    EXPECT_NEAR(start, 0.2, 0.005);

    cell.devices().find<robotik::ConveyorBelt>("feeder_belt")->run(true);
    run(cell, 1.0);
    // Pulled to the belt speed in a few hundredths of a second.
    double const moved = cell.pose("cube").position.x - start;
    EXPECT_GT(moved, 0.9 * SPEED);
    EXPECT_LT(moved, 1.02 * SPEED);
    EXPECT_NEAR(cell.pose("cube").position.y, -0.8, 0.005);
    EXPECT_NEAR(cell.pose("cube").position.z, 0.12, 0.005);
}

TEST(ConveyorBelt, AFailedBeltStops)
{
    compages::world::World world;
    robotik::Cell cell(world, beltCell(robotik::Fidelity::Abstract));
    cell.devices().find<robotik::ConveyorBelt>("feeder_belt")->run(true);
    cell.robot().resources().fail("feeder_belt");
    run(cell, 1.0);
    EXPECT_NEAR(cell.pose("cube").position.x, 0.2, 1e-6);
}

/// A world of an arm and a belt, in YAML.
static char const* const BELT_WORLD = R"(
id: belt_world
entities:
  - id: arm
    type: robot
    model: robotik.robots.robot_6axis
  - id: feeder
    type: body
    physics: fixed
    shape: { type: box, size: [1.0, 0.2, 0.1] }
    actuators:
      feeder_belt: { type: conveyor, speed: 0.3, direction: [0, 2, 0] }
)";

/// The error of a world with the @p_path node replaced by @p_yaml.
static std::string worldError(std::string const& p_path, std::string const& p_yaml)
{
    robotik::ConfigNode node = robotik::ConfigNode::parse(BELT_WORLD).take();
    EXPECT_TRUE(node.set(p_path, robotik::ConfigNode::parse(p_yaml).take()));
    auto const world = robotik::readWorld(node, ".", testAssets());
    return world ? std::string() : world.error();
}

TEST(ConveyorBelt, WorldReadsTheBelt)
{
    robotik::ConfigNode const node = robotik::ConfigNode::parse(BELT_WORLD).take();
    robotik::WorldDescription const world = robotik::readWorld(node, ".", testAssets()).take();
    robotik::EntitySpec const* feeder = robotik::findEntity(world, "feeder");
    ASSERT_NE(feeder, nullptr);
    ASSERT_EQ(feeder->actuators.size(), 1U);
    robotik::ActuatorSpec const& belt = feeder->actuators[0];
    EXPECT_EQ(belt.type, robotik::ActuatorSpec::Type::Conveyor);
    EXPECT_DOUBLE_EQ(belt.speed.value(), 0.3);
    EXPECT_DOUBLE_EQ(belt.direction.y, 2.0);

    // The cell normalizes the direction.
    robotik::CellDescription cell;
    cell.world = world;
    compages::world::World ecs;
    robotik::Cell built(ecs, cell);
    auto const* device = built.devices().find<robotik::ConveyorBelt>("feeder_belt");
    ASSERT_NE(device, nullptr);
    EXPECT_DOUBLE_EQ(device->direction().y, 1.0);
}

TEST(ConveyorBelt, WorldRefusesABadBelt)
{
    EXPECT_NE(worldError("entities.feeder.physics", "free").find("on a fixed box"),
              std::string::npos);
    EXPECT_NE(worldError("entities.feeder.actuators.feeder_belt",
                         "{ type: conveyor, speed: 0 }")
                  .find("speed"),
              std::string::npos);
    EXPECT_NE(worldError("entities.feeder.actuators.feeder_belt",
                         "{ type: conveyor, direction: [0, 0, 1] }")
                  .find("direction needs x or y"),
              std::string::npos);
    EXPECT_NE(worldError("entities.arm.actuators", "{ belt: { type: conveyor } }")
                  .find("not of a robot"),
              std::string::npos);
}
