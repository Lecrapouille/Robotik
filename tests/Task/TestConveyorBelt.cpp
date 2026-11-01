// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

// The conveyor belt, a device of the cell: its skill, the parts it carries
// with and without physics, its stop, a part put on it, a failed belt.

#include "main.hpp"

#include "Robotik/Common/Config.hpp"
#include "Robotik/Package/Scenario/World.hpp"
#include "Robotik/Robot/Components/Objects.hpp"
#include "Robotik/Task/Cell.hpp"

#include "Compages/World/World.hpp"

/// Speed of the belt (m/s).
static constexpr double SPEED = 0.2;

/// An arm away from a belt of 1 m along +x; a cube resting on the belt.
/// With @p_stop, a stop at the end of the belt (x = 1).
static robotik::CellDescription beltCell(robotik::Fidelity p_fidelity,
                                         bool p_stop = false)
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
    drive.stop = p_stop;
    belt.actuators = { drive };

    robotik::EntitySpec cube;
    cube.id = "cube";
    cube.shape = robotik::ShapeSpec{};
    cube.pose.position = robotik::Vector3(0.2, -0.8, 0.12);
    cell.world.entities = { arm, belt, cube };
    EXPECT_TRUE(robotik::addBeltStops(cell.world));
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
    // Without physics the part stops at the end of the belt (1 m), on it.
    EXPECT_NEAR(cell.pose("cube").position.x, 1.0 - 0.02, 1e-6);
    EXPECT_NEAR(cell.pose("cube").position.z, 0.12, 1e-6);
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

TEST(ConveyorBelt, ItsStopHoldsARigidCube)
{
    compages::world::World world;
    robotik::CellDescription const description =
        beltCell(robotik::Fidelity::Rigid, true);
    robotik::EntitySpec const* stop =
        robotik::findEntity(description.world, "feeder_stop");
    ASSERT_NE(stop, nullptr);
    EXPECT_EQ(stop->physics, robotik::EntitySpec::Physics::Fixed);
    robotik::Cell cell(world, description);
    run(cell, 0.3);
    cell.devices().find<robotik::ConveyorBelt>("feeder_belt")->run(true);
    run(cell, 5.0);
    // Against the stop at the end of the belt, on it; the belt slips under it.
    EXPECT_NEAR(cell.pose("cube").position.x, 1.0 - 0.02, 0.005);
    EXPECT_NEAR(cell.pose("cube").position.z, 0.12, 0.005);

    // Without the stop the cube falls at the end.
    compages::world::World open;
    robotik::Cell fall(open, beltCell(robotik::Fidelity::Rigid));
    run(fall, 0.3);
    fall.devices().find<robotik::ConveyorBelt>("feeder_belt")->run(true);
    run(fall, 5.0);
    EXPECT_LT(fall.pose("cube").position.z, 0.05);
}

TEST(ConveyorBelt, AStoppedBeltHoldsASlidingCube)
{
    compages::world::World world;
    robotik::Cell cell(world, beltCell(robotik::Fidelity::Rigid));
    run(cell, 0.3);
    // A short run, then the belt stops: the friction of the belt stops the
    // cube at once (it does not slide on, as on ice).
    auto* belt = cell.devices().find<robotik::ConveyorBelt>("feeder_belt");
    belt->run(true);
    run(cell, 0.5);
    belt->run(false);
    run(cell, 0.2);
    double const stopped = cell.pose("cube").position.x;
    run(cell, 1.0);
    EXPECT_NEAR(cell.pose("cube").position.x, stopped, 0.002);
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

TEST(ConveyorBelt, WorldAddsTheStopAtTheEndOfTheBelt)
{
    robotik::ConfigNode node = robotik::ConfigNode::parse(BELT_WORLD).take();
    ASSERT_TRUE(node.set("entities.feeder.actuators.feeder_belt.stop",
                         robotik::ConfigNode::parse("true").take()));
    robotik::WorldDescription const world =
        robotik::readWorld(node, ".", testAssets()).take();
    // The belt goes along +y: the stop is across it, just past y = 0.5.
    robotik::EntitySpec const* stop = robotik::findEntity(world, "feeder_stop");
    ASSERT_NE(stop, nullptr);
    EXPECT_EQ(stop->type, robotik::EntitySpec::Type::Body);
    EXPECT_EQ(stop->physics, robotik::EntitySpec::Physics::Fixed);
    ASSERT_TRUE(stop->shape);
    EXPECT_NEAR(stop->shape->size.x, 0.01, 1e-9);
    EXPECT_NEAR(stop->shape->size.y, 1.0, 1e-9);
    EXPECT_NEAR(stop->shape->size.z, 0.12, 1e-9);
    EXPECT_NEAR(stop->pose.position.x, 0.0, 1e-9);
    EXPECT_NEAR(stop->pose.position.y, 0.105, 1e-9);
    EXPECT_NEAR(stop->pose.position.z, 0.01, 1e-9);
    // Its x axis along the belt.
    robotik::Vector3 const along =
        stop->pose.rotation * robotik::Vector3(1.0, 0.0, 0.0);
    EXPECT_NEAR(along.y, 1.0, 1e-9);

    // Its id is taken.
    EXPECT_NE(
        worldError(
            "entities",
            "[ { id: arm, type: robot, model: robotik.robots.robot_6axis },"
            "  { id: feeder, type: body, physics: fixed,"
            "    shape: { type: box, size: [1, 0.2, 0.1] },"
            "    actuators: { b: { type: conveyor, stop: true } } },"
            "  { id: feeder_stop, type: prop, shape: { type: box } } ]")
            .find("stop of a belt"),
        std::string::npos);
}

/// A world of an arm, a conveyor and a buffer written with the shorthands.
static char const* const SHORTHAND_WORLD = R"(
id: shorthand_world
entities:
  - id: arm
    type: robot
    model: robotik.robots.robot_6axis
  - id: feeder
    type: conveyor
    size: [1.0, 0.2, 0.1]
    pose: { position: [0.5, -0.8, 0.05] }
    speed: 0.3
    direction: [0, 1, 0]
    stop: true
  - id: station
    type: buffer
    kind: bin
    places: 3
    pitch: 0.2
    direction: [0, 1, 0]
    colors: [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    pose: { position: [0.4, 0.0, 0.005] }
)";

TEST(ConveyorBelt, WorldExpandsTheConveyorAndTheBufferShorthands)
{
    robotik::ConfigNode const node =
        robotik::ConfigNode::parse(SHORTHAND_WORLD).take();
    robotik::WorldDescription const world =
        robotik::readWorld(node, ".", testAssets()).take();

    // The conveyor: a fixed box body, its belt <id>_belt, its stop.
    robotik::EntitySpec const* feeder = robotik::findEntity(world, "feeder");
    ASSERT_NE(feeder, nullptr);
    EXPECT_EQ(feeder->type, robotik::EntitySpec::Type::Body);
    EXPECT_EQ(feeder->physics, robotik::EntitySpec::Physics::Fixed);
    ASSERT_TRUE(feeder->shape);
    EXPECT_DOUBLE_EQ(feeder->shape->size.x, 1.0);
    ASSERT_EQ(feeder->actuators.size(), 1U);
    EXPECT_EQ(feeder->actuators[0].name, "feeder_belt");
    EXPECT_EQ(feeder->actuators[0].type, robotik::ActuatorSpec::Type::Conveyor);
    EXPECT_DOUBLE_EQ(feeder->actuators[0].speed.value(), 0.3);
    EXPECT_TRUE(feeder->actuators[0].stop);
    EXPECT_NE(robotik::findEntity(world, "feeder_stop"), nullptr);

    // The buffer: three containers along y, 20 cm apart, around its pose.
    EXPECT_EQ(robotik::findEntity(world, "station"), nullptr);
    double const y[] = { -0.2, 0.0, 0.2 };
    for (std::size_t i = 0U; i < 3U; ++i)
    {
        robotik::EntitySpec const* place =
            robotik::findEntity(world, "station_" + std::to_string(i + 1U));
        ASSERT_NE(place, nullptr) << i;
        EXPECT_EQ(place->type, robotik::EntitySpec::Type::Prop);
        EXPECT_EQ(place->kind, "bin");
        ASSERT_TRUE(place->shape);
        EXPECT_EQ(place->shape->type, robotik::ShapeSpec::Type::Container);
        EXPECT_NEAR(place->pose.position.x, 0.4, 1e-9);
        EXPECT_NEAR(place->pose.position.y, y[i], 1e-9);
        EXPECT_FLOAT_EQ(place->shape->color[i], 1.0F);
    }

    // A cell runs the belt of the shorthand.
    robotik::CellDescription cell;
    cell.world = world;
    compages::world::World ecs;
    robotik::Cell built(ecs, cell);
    EXPECT_NE(built.devices().find<robotik::ConveyorBelt>("feeder_belt"),
              nullptr);
}

/// The error of the shorthand world with the @p_path node replaced by @p_yaml.
static std::string shorthandError(std::string const& p_path,
                                  std::string const& p_yaml)
{
    robotik::ConfigNode node =
        robotik::ConfigNode::parse(SHORTHAND_WORLD).take();
    EXPECT_TRUE(node.set(p_path, robotik::ConfigNode::parse(p_yaml).take()));
    auto const world = robotik::readWorld(node, ".", testAssets());
    return world ? std::string() : world.error();
}

TEST(ConveyorBelt, WorldRefusesABadShorthand)
{
    EXPECT_NE(shorthandError("entities.feeder.size", "~").find("needs a size"),
              std::string::npos);
    EXPECT_NE(shorthandError("entities.feeder.shape", "{ type: box }")
                  .find("not a shape"),
              std::string::npos);
    EXPECT_NE(shorthandError("entities.feeder.speed", "0").find("speed"),
              std::string::npos);
    EXPECT_NE(
        shorthandError("entities.station.places", "0").find("whole number"),
        std::string::npos);
    EXPECT_NE(
        shorthandError("entities.station.places", "2.5").find("whole number"),
        std::string::npos);
    EXPECT_NE(shorthandError("entities.station.colors", "[[1, 0, 0]]")
                  .find("one [r, g, b]"),
              std::string::npos);
    // station_2 is one of its places: the robot cannot take its id.
    robotik::ConfigNode node =
        robotik::ConfigNode::parse(SHORTHAND_WORLD).take();
    ASSERT_TRUE(node.set("entities.arm.id",
                         robotik::ConfigNode::parse("station_2").take()));
    EXPECT_FALSE(robotik::readWorld(node, ".", testAssets()));
}

TEST(ConveyorBelt, APartPutOnABeltLandsOnItWithoutPhysics)
{
    compages::world::World world;
    robotik::CellDescription description =
        beltCell(robotik::Fidelity::Abstract);
    robotik::EntitySpec cup;
    cup.id = "cup";
    cup.type = robotik::EntitySpec::Type::Tool;
    cup.model = repoFile("data/packs/robotik.tools/tool_vacuum.urdf");
    cup.attach = robotik::Attachment{ "arm", "flange", "tool_mount", {} };
    robotik::ActuatorSpec suction;
    suction.name = "gripper";
    suction.type = robotik::ActuatorSpec::Type::Vacuum;
    cup.actuators = { suction };
    description.world.entities.push_back(cup);
    robotik::Cell cell(world, description);
    auto* gripper = cell.robot().actuators().first<robotik::VacuumGripper>();
    ASSERT_NE(gripper, nullptr);

    run(cell, 0.05);
    // The cube held 5 cm above the belt (top at z = 0.1), then let go.
    compages::world::Entity cube;
    world.each<robotik::ecs::Shape>(
        [&cube](compages::world::Entity p_entity,
                robotik::ecs::Shape const& p_shape)
        {
            if (p_shape.name == "cube")
            {
                cube = p_entity;
            }
        });
    ASSERT_TRUE(cube);
    gripper->held(cube.id());
    cube.position(0.6F, -0.8F, 0.17F);
    gripper->suction(false);
    run(cell, 0.05);
    EXPECT_NEAR(cell.pose("cube").position.z, 0.12, 1e-4);
    EXPECT_NEAR(cell.pose("cube").position.x, 0.6, 1e-4);
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
