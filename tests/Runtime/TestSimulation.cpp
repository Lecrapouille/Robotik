// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "main.hpp"

#include "PickPlaceMission.hpp"

#include "Robotik/Runtime/Simulation.hpp"

#include "Compages/World/World.hpp"

namespace
{

struct Outcome
{
    bool passed = true;
    double time = 0.0;
    std::vector<std::string> runs;
};

Outcome run(robotik::Simulation& p_simulation, robotik::Seed p_seed)
{
    p_simulation.reset(p_seed);
    while (!p_simulation.finished() && p_simulation.time() < Seconds(60.0))
    {
        p_simulation.step(Seconds(0.01));
    }
    Outcome outcome;
    for (auto const& check : p_simulation.checks())
    {
        outcome.passed = outcome.passed && check.passed;
    }
    outcome.time = p_simulation.time().value();
    robotik::SkillScheduler const& skills = p_simulation.skills();
    for (robotik::SkillRun const& entry : skills.trace())
    {
        outcome.runs.push_back(skills.name(entry.skill) + ":" +
                               robotik::toString(entry.state));
    }
    return outcome;
}

} // namespace

TEST(Scenario, LoadsSensorsActuatorsObjectsAndFaults)
{
    robotik::Scenario const scenario =
        robotik::Scenario::load(dataFile("scenarios/pick_and_place_faults.yml"));
    EXPECT_EQ(scenario.seed, 7u);
    ASSERT_EQ(scenario.cameras.size(), 1u);
    EXPECT_EQ(scenario.cameras[0].name, "wrist_camera");
    EXPECT_EQ(scenario.cameras[0].config.parent, "link6");
    ASSERT_EQ(scenario.actuators.size(), 2u);
    EXPECT_EQ(scenario.actuators[1].type, robotik::Scenario::Actuator::Type::Vacuum);
    ASSERT_EQ(scenario.objects.size(), 2u);
    EXPECT_EQ(scenario.objects[0].shape.name, "red_cube");
    EXPECT_DOUBLE_EQ(scenario.objects[0].randomize[0][1], 0.02);
    ASSERT_EQ(scenario.faults.size(), 3u);
    EXPECT_FALSE(scenario.faults[1].disable);
}

TEST(Simulation, PickAndPlaceSucceedsAndReplays)
{
    PickPlaceMission mission;
    compages::world::World world;
    robotik::Simulation simulation(
        world,
        robotik::Scenario::load(dataFile("scenarios/pick_and_place.yml")),
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
        robotik::Scenario::load(dataFile("scenarios/pick_and_place_faults.yml")),
        nullptr,
        &mission);
    simulation.reset(robotik::Seed{ 7 });
    robotik::SkillId const detect = simulation.skills().find("Detect(red_cube)");
    ASSERT_NE(detect, robotik::NO_SKILL);
    while (simulation.time() < Seconds(0.99))
    {
        simulation.step(Seconds(0.01));
        EXPECT_NE(simulation.skills().state(detect),
                  robotik::SkillState::Succeeded);
    }
    while (simulation.time() < Seconds(1.05))
    {
        simulation.step(Seconds(0.01));
    }
    EXPECT_EQ(simulation.skills().state(detect), robotik::SkillState::Succeeded);
}

TEST(Simulation, SurvivesACameraFailure)
{
    PickPlaceMission mission;
    compages::world::World world;
    robotik::Simulation simulation(
        world,
        robotik::Scenario::load(dataFile("scenarios/pick_and_place_faults.yml")),
        nullptr,
        &mission);
    Outcome const outcome = run(simulation, robotik::Seed{ 7 });
    EXPECT_TRUE(outcome.passed);
    EXPECT_FALSE(simulation.robot().resources().available("wrist_camera"));
}

TEST(Simulation, EmergencyStopPreemptsTheArm)
{
    PickPlaceMission mission;
    compages::world::World world;
    robotik::Simulation simulation(
        world,
        robotik::Scenario::load(dataFile("scenarios/pick_and_place.yml")),
        nullptr,
        &mission);
    robotik::SkillScheduler& skills = simulation.skills();
    robotik::SkillId const stop = skills.find("Stop");
    ASSERT_NE(stop, robotik::NO_SKILL);

    for (int i = 0; i < 100; ++i)
    {
        simulation.step(Seconds(0.01));
    }
    robotik::SkillId const approach = skills.find("Approach(red_cube)");
    ASSERT_EQ(skills.state(approach), robotik::SkillState::Running);

    skills.request(stop);
    simulation.step(Seconds(0.01));
    EXPECT_EQ(skills.state(stop), robotik::SkillState::Running);
    EXPECT_EQ(skills.state(approach), robotik::SkillState::Cancelled);
    EXPECT_EQ(skills.reason(approach), robotik::SkillReason::Preempted);
}
