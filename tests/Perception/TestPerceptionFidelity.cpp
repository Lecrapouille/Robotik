// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "main.hpp"

#include "PickPlaceMission.hpp"

#include "Robotik/Package/RunReport.hpp"
#include "Robotik/Package/Simulation.hpp"

#include "Compages/World/World.hpp"

#include <string>

/// The pick-and-place scenario with some values changed before it is read.
static robotik::Scenario
pickAndPlace(char const* p_file,
             std::vector<std::pair<char const*, char const*>> const& p_changes)
{
    robotik::ScenarioSource source = robotik::ScenarioSource::load(repoFile(p_file)).take();
    for (auto const& [path, value] : p_changes)
    {
        source.override(path, robotik::ConfigNode(value));
    }
    return robotik::Scenario::read(source, testAssets()).take();
}

/// Steps until the run ends or @p_limit seconds.
static void runFor(robotik::Simulation& p_simulation, double p_limit)
{
    while (!p_simulation.finished() && p_simulation.time() < robotik::Seconds(p_limit))
    {
        p_simulation.step(robotik::Seconds(0.01));
    }
    p_simulation.conclude();
}

// ----------------------------------------------------------------------------
TEST(PerceptionFidelity, LegacyOracleKeyStillWorks)
{
    // A scenario without a perception section: only the old key is set.
    robotik::Scenario const always = pickAndPlace(
        "plugins/HelloWorld/scenarios/hello.scenario.yaml",
        { { "scenario.perception.oracle", "always" } });
    EXPECT_EQ(always.perception.fidelity, robotik::PerceptionFidelity::Truth);
    robotik::Scenario const blind = pickAndPlace(
        "plugins/HelloWorld/scenarios/hello.scenario.yaml",
        { { "scenario.perception.oracle", "when_blind" } });
    EXPECT_EQ(blind.perception.fidelity, robotik::PerceptionFidelity::Sensed);
    EXPECT_EQ(blind.perception.blind, robotik::PerceptionFidelity::Truth);
}

// ----------------------------------------------------------------------------
TEST(PerceptionFidelity, EachObjectSaysWhereItsBeliefComesFrom)
{
    PickPlaceMission mission;
    compages::world::World world;
    robotik::Simulation simulation(
        world,
        pickAndPlace("plugins/PickAndPlace/scenarios/pick_and_place.scenario.yaml",
                     { { "scenario.perception.blind", "sensed" },
                       { "scenario.perception.fidelity", "truth" },
                       { "scenario.perception.overrides.box", "sensed" } }),
        nullptr,
        &mission);
    EXPECT_EQ(simulation.cell().perceptionOf("red_cube"), robotik::PerceptionFidelity::Truth);
    EXPECT_EQ(simulation.cell().perceptionOf("box"), robotik::PerceptionFidelity::Sensed);
    simulation.step(robotik::Seconds(0.01));
    robotik::WorldObject const* cube = simulation.cell().worldModel().find("red_cube");
    ASSERT_NE(cube, nullptr);
    EXPECT_EQ(cube->source, robotik::BeliefSource::Truth);
    EXPECT_EQ(simulation.cell().worldModel().find("box")->source, robotik::BeliefSource::Prior);

    // The report says it: a mocked perception is never hidden.
    runFor(simulation, 1.0);
    robotik::RunRecord const record = robotik::RunRecord::capture(simulation, "case", {});
    EXPECT_TRUE(record.oracle);
    EXPECT_NE(record.perception.find("truth"), std::string::npos) << record.perception;
    EXPECT_NE(record.perception.find("box: sensed"), std::string::npos) << record.perception;
}

// ----------------------------------------------------------------------------
TEST(PerceptionFidelity, NoisyTruthCanMissAnObject)
{
    PickPlaceMission mission;
    compages::world::World world;
    robotik::Simulation simulation(
        world,
        pickAndPlace("plugins/PickAndPlace/scenarios/pick_and_place.scenario.yaml",
                     { { "scenario.perception.blind", "sensed" },
                       { "scenario.perception.fidelity", "noisy_truth" },
                       { "scenario.perception.noise.dropout", "1.0" } }),
        nullptr,
        &mission);
    for (int i = 0; i < 50; ++i)
    {
        simulation.step(robotik::Seconds(0.01));
    }
    EXPECT_FALSE(simulation.cell().worldModel().find("red_cube")->observed());
}

// ----------------------------------------------------------------------------
TEST(PerceptionFidelity, ObservedEffectsNeedAFreshObservation)
{
    // With the true poses, every effect is seen again: the plan succeeds.
    {
        PickPlaceMission mission(true, true);
        compages::world::World world;
        robotik::Simulation simulation(
            world,
            pickAndPlace("plugins/PickAndPlace/scenarios/pick_and_place_pddl.scenario.yaml",
                         { { "scenario.controller.planner.verify", "observed" } }),
            nullptr,
            &mission);
        ASSERT_TRUE(simulation.cell().planner()->spec().observed);
        runFor(simulation, 60.0);
        EXPECT_EQ(simulation.outcome(), robotik::Simulation::Outcome::ControllerSucceeded);
    }
    // Headless without the true poses, nothing is observed: no step may claim
    // its effects, so the plan cannot succeed.
    {
        PickPlaceMission mission(true, true);
        compages::world::World world;
        robotik::Simulation simulation(
            world,
            pickAndPlace("plugins/PickAndPlace/scenarios/pick_and_place_pddl.scenario.yaml",
                         { { "scenario.controller.planner.verify", "observed" },
                           { "scenario.perception.blind", "sensed" } }),
            nullptr,
            &mission);
        runFor(simulation, 60.0);
        EXPECT_NE(simulation.outcome(), robotik::Simulation::Outcome::ControllerSucceeded);
    }
}
