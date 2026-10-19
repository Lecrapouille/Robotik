// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "main.hpp"

#include "PickPlaceMission.hpp"

#include "Robotik/Package/Simulation.hpp"

#include "Compages/World/World.hpp"

#include <algorithm>

/// How a short run finished: verdict, time and the skill trace.
struct Outcome
{
    bool passed = true;
    double time = 0.0;
    std::vector<std::string> runs;
};

/// Resets, steps until the run ends, and returns how it finished.
static Outcome run(robotik::Simulation& p_simulation, robotik::Seed p_seed)
{
    p_simulation.reset(p_seed);
    while (!p_simulation.finished() && p_simulation.time() < robotik::Seconds(60.0))
    {
        p_simulation.step(robotik::Seconds(0.01));
    }
    p_simulation.conclude();
    Outcome outcome;
    outcome.passed = p_simulation.passed();
    outcome.time = p_simulation.time().value();
    robotik::SkillScheduler const& skills = p_simulation.skills();
    for (robotik::SkillRun const& entry : skills.trace())
    {
        outcome.runs.push_back(skills.name(entry.skill) + ":" +
                               robotik::toString(entry.state));
    }
    return outcome;
}

TEST(Scenario, LoadsWorldEntitiesFaultsAndAssertions)
{
    robotik::Scenario const scenario =
        robotik::Scenario::load(repoFile("plugins/PickAndPlace/scenarios/"
                                         "pick_and_place_faults.scenario.yaml"),
                                testAssets());
    EXPECT_EQ(scenario.id, "pick_and_place_faults");
    EXPECT_EQ(scenario.seed, 7u);
    EXPECT_EQ(scenario.controller.behavior_tree.filename(),
              "pick_and_place.bt.yaml");
    EXPECT_DOUBLE_EQ(scenario.timeout.value(), 60.0);

    robotik::WorldDescription const& world = scenario.world;
    EXPECT_EQ(world.id, "pick_place_cell");
    robotik::EntitySpec const& robot = world.robot();
    ASSERT_EQ(robot.sensors.size(), 1u);
    EXPECT_EQ(robot.sensors[0].name, "wrist_camera");
    EXPECT_EQ(robot.sensors[0].camera.parent, "link6");
    ASSERT_EQ(robot.actuators.size(), 1u);

    // The vacuum cup is a tool of its own, with its actuator; four other
    // tools wait in storage.
    robotik::EntitySpec const* tool = world.mountedTool();
    ASSERT_NE(tool, nullptr);
    EXPECT_EQ(tool->id, "vacuum");
    ASSERT_EQ(tool->actuators.size(), 1u);
    EXPECT_EQ(tool->actuators[0].type, robotik::ActuatorSpec::Type::Vacuum);
    EXPECT_EQ(std::count_if(
                  world.entities.begin(),
                  world.entities.end(),
                  [](robotik::EntitySpec const& p_entity)
                  { return p_entity.type == robotik::EntitySpec::Type::Tool; }),
              5);

    robotik::EntitySpec const* cube = world.find("red_cube");
    ASSERT_NE(cube, nullptr);
    EXPECT_EQ(cube->type, robotik::EntitySpec::Type::Prop);
    EXPECT_DOUBLE_EQ(cube->randomize[0][1], 0.02);
    EXPECT_EQ(world.find("box")->shape->type,
              robotik::ShapeSpec::Type::Container);

    ASSERT_EQ(scenario.faults.size(), 3u);
    EXPECT_FALSE(scenario.faults[1].disable);
    ASSERT_EQ(scenario.assertions.size(), 5u);
    EXPECT_EQ(scenario.assertions[3].phase, robotik::AssertionPhase::Always);
}

TEST(Scenario, OverridesChangeTheWorldBeforeItIsRead)
{
    robotik::ScenarioSource source = robotik::ScenarioSource::load(repoFile(
        "plugins/PickAndPlace/scenarios/pick_and_place.scenario.yaml"));
    source.override("world.entities.red_cube.pose.position",
                    robotik::ConfigNode::parse("[0.30, 0.10, 0.02]"));
    source.override("world.entities.robot.sensors.wrist_camera.noise",
                    robotik::ConfigNode("0.05"));
    source.override("scenario.seed", robotik::ConfigNode("3"));
    robotik::Scenario const scenario =
        robotik::Scenario::read(source, testAssets());
    EXPECT_EQ(scenario.seed, 3u);
    EXPECT_DOUBLE_EQ(scenario.world.find("red_cube")->pose.position.x, 0.30);
    EXPECT_DOUBLE_EQ(scenario.world.robot().sensors[0].camera.noise, 0.05);
    EXPECT_THROW(
        source.override("world.entities.nobody.mass", robotik::ConfigNode("1")),
        std::runtime_error);
    EXPECT_THROW(source.override("seed", robotik::ConfigNode("1")),
                 std::runtime_error);
}

TEST(Scenario, WorldErrorsNameTheEntity)
{
    robotik::ScenarioSource source = robotik::ScenarioSource::load(repoFile(
        "plugins/PickAndPlace/scenarios/pick_and_place.scenario.yaml"));
    source.override("world.entities.box.type", robotik::ConfigNode("robot"));
    try
    {
        (void)robotik::Scenario::read(source, testAssets());
        FAIL() << "a second robot must be refused";
    }
    catch (std::runtime_error const& error)
    {
        EXPECT_NE(std::string(error.what()).find("pick_place_cell.world.yaml"),
                  std::string::npos)
            << error.what();
    }
}

TEST(Simulation, AnotherToolBringsItsOwnActuators)
{
    robotik::Scenario scenario = robotik::Scenario::load(
        repoFile("plugins/PickAndPlace/scenarios/pick_and_place.scenario.yaml"),
        testAssets());
    scenario.world.mount("fingers");
    PickPlaceMission mission;
    compages::world::World world;
    robotik::Simulation simulation(
        world, std::move(scenario), nullptr, &mission);
    EXPECT_EQ(simulation.robot().actuators().find("gripper"), nullptr);
    EXPECT_EQ(simulation.query("tool_is_attached(fingers, robot.flange)"), 1.0);
    EXPECT_EQ(simulation.query("tool_is_attached(vacuum, robot.flange)"), 0.0);
    EXPECT_THROW((void)simulation.pose("vacuum"), std::invalid_argument);
}

TEST(Simulation, PickAndPlaceSucceedsAndReplays)
{
    PickPlaceMission mission;
    compages::world::World world;
    robotik::Simulation simulation(
        world,
        robotik::Scenario::load(
            repoFile(
                "plugins/PickAndPlace/scenarios/pick_and_place.scenario.yaml"),
            testAssets()),
        nullptr,
        &mission);

    Outcome const first = run(simulation, robotik::Seed{ 11 });
    EXPECT_TRUE(first.passed);
    Outcome const other = run(simulation, robotik::Seed{ 12 });
    EXPECT_TRUE(other.passed);
    Outcome const again = run(simulation, robotik::Seed{ 11 });
    EXPECT_EQ(first.runs, again.runs);
    EXPECT_DOUBLE_EQ(first.time, again.time);
}

TEST(Simulation, DetectWaitsUntilCameraIsRestored)
{
    PickPlaceMission mission;
    compages::world::World world;
    robotik::Simulation simulation(
        world,
        robotik::Scenario::load(repoFile("plugins/PickAndPlace/scenarios/"
                                         "pick_and_place_faults.scenario.yaml"),
                                testAssets()),
        nullptr,
        &mission);
    simulation.reset(robotik::Seed{ 7 });
    robotik::SkillId const detect =
        simulation.skills().find("Detect(red_cube)");
    ASSERT_NE(detect, robotik::NO_SKILL);
    while (simulation.time() < robotik::Seconds(0.99))
    {
        simulation.step(robotik::Seconds(0.01));
        EXPECT_NE(simulation.skills().state(detect),
                  robotik::SkillState::Succeeded);
    }
    while (simulation.time() < robotik::Seconds(1.05))
    {
        simulation.step(robotik::Seconds(0.01));
    }
    EXPECT_EQ(simulation.skills().state(detect),
              robotik::SkillState::Succeeded);
}

TEST(Simulation, SurvivesACameraFailure)
{
    PickPlaceMission mission;
    compages::world::World world;
    robotik::Simulation simulation(
        world,
        robotik::Scenario::load(repoFile("plugins/PickAndPlace/scenarios/"
                                         "pick_and_place_faults.scenario.yaml"),
                                testAssets()),
        nullptr,
        &mission);
    Outcome const outcome = run(simulation, robotik::Seed{ 7 });
    EXPECT_TRUE(outcome.passed);
    EXPECT_FALSE(simulation.robot().resources().available("wrist_camera"));
}

TEST(Simulation, FailedAssertionDoesNotStopTheRun)
{
    PickPlaceMission mission;
    compages::world::World world;
    robotik::Simulation simulation(
        world,
        robotik::Scenario::load(
            repoFile(
                "plugins/PickAndPlace/scenarios/pick_and_place.scenario.yaml"),
            testAssets()),
        nullptr,
        &mission);
    simulation.reset(robotik::Seed{ 11 });
    simulation.step(robotik::Seconds(0.01));
    bool failed = false;
    for (robotik::Check const& check : simulation.checks())
    {
        failed = failed || !check.ok();
    }
    EXPECT_TRUE(failed);
    EXPECT_FALSE(simulation.finished());
    simulation.step(robotik::Seconds(0.01));
    EXPECT_FALSE(simulation.finished());
}

TEST(Simulation, EmergencyStopPreemptsTheArm)
{
    PickPlaceMission mission;
    compages::world::World world;
    robotik::Simulation simulation(
        world,
        robotik::Scenario::load(
            repoFile(
                "plugins/PickAndPlace/scenarios/pick_and_place.scenario.yaml"),
            testAssets()),
        nullptr,
        &mission);
    robotik::SkillScheduler& skills = simulation.skills();
    robotik::SkillId const stop = skills.find("Stop");
    ASSERT_NE(stop, robotik::NO_SKILL);

    for (int i = 0; i < 100; ++i)
    {
        simulation.step(robotik::Seconds(0.01));
    }
    robotik::SkillId const approach = skills.find("Approach(red_cube)");
    ASSERT_EQ(skills.state(approach), robotik::SkillState::Running);

    skills.request(stop);
    simulation.step(robotik::Seconds(0.01));
    EXPECT_EQ(skills.state(stop), robotik::SkillState::Running);
    EXPECT_EQ(skills.state(approach), robotik::SkillState::Cancelled);
    EXPECT_EQ(skills.reason(approach), robotik::SkillReason::Preempted);
}
