// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

// The GRAFCET controller: scenario YAML, then the GRAFCET scenarios of the
// pick and place demo, end to end and headless.

#include "main.hpp"

#include "PickPlaceMission.hpp"

#include "Robotik/Package/Scenario/Scenario.hpp"
#include "Robotik/Package/Simulation.hpp"
#include "Robotik/Task/Grafcet/GrafcetController.hpp"

#include "Compages/World/World.hpp"

#include <algorithm>

static char const* const SEQUENCE =
    "plugins/PickAndPlace/scenarios/pick_and_place_grafcet.scenario.yaml";
static char const* const GEMMA =
    "plugins/PickAndPlace/scenarios/pick_and_place_gemma.scenario.yaml";
static char const* const GEMMA_STOP =
    "plugins/PickAndPlace/scenarios/pick_and_place_gemma_stop.scenario.yaml";
static char const* const GEMMA_REARM =
    "plugins/PickAndPlace/scenarios/pick_and_place_gemma_rearm.scenario.yaml";

/// The scenario source of a file, to edit before reading it.
static robotik::ScenarioSource source(char const* p_file)
{
    return robotik::loadScenarioSource(repoFile(p_file)).take();
}

/// A YAML node from text.
static robotik::ConfigNode yaml(std::string const& p_text)
{
    return robotik::ConfigNode::parse(p_text).take();
}

TEST(GrafcetScenario, ReadsTheGrafcetBlock)
{
    robotik::Scenario const scenario =
        robotik::loadScenario(repoFile(GEMMA_STOP), testAssets()).take();
    ASSERT_TRUE(scenario.controller.grafcet.has_value());
    robotik::GrafcetSpec const& spec = *scenario.controller.grafcet;
    EXPECT_EQ(spec.file.filename(), "pick_and_place_gemma.grafcet.json");
    EXPECT_EQ(spec.success, "Safety:X1 . 2s/Safety:X1 . Control:X0 . Production:X0");
    EXPECT_TRUE(spec.inputs.empty());
    ASSERT_EQ(spec.events.size(), 2U);
    EXPECT_EQ(spec.events[0].input, "Dcy");
    EXPECT_TRUE(spec.events[0].pulse);
    EXPECT_DOUBLE_EQ(spec.events[1].at.value(), 3.0);
    EXPECT_EQ(spec.events[1].input, "AU");
    EXPECT_FALSE(spec.events[1].pulse);
    EXPECT_TRUE(scenario.controller.behavior_tree.empty());
    EXPECT_FALSE(scenario.controller.planner.has_value());
}

TEST(GrafcetScenario, RefusesTwoControllersAndOtherFiles)
{
    robotik::ScenarioSource two = source(SEQUENCE);
    ASSERT_TRUE(two.scenario.set("scenario.controller.behavior_tree",
                                 yaml("scenarios/pick_and_place.bt.yaml")));
    auto const read = robotik::readScenario(two, testAssets());
    ASSERT_FALSE(read);
    EXPECT_NE(read.error().find("not several"), std::string::npos) << read.error();

    robotik::ScenarioSource bad = source(SEQUENCE);
    ASSERT_TRUE(bad.scenario.set("scenario.controller.grafcet",
                                 yaml("scenarios/pick_and_place.bt.yaml")));
    auto const wrong = robotik::readScenario(bad, testAssets());
    ASSERT_FALSE(wrong);
    EXPECT_NE(wrong.error().find("not a .grafcet.json"), std::string::npos)
        << wrong.error();
}

/// Runs a scenario until it ends; every assertion must pass.
static robotik::Simulation::Outcome
runToTheEnd(robotik::Simulation& p_simulation, double p_limit)
{
    p_simulation.reset(robotik::Seed{ p_simulation.scenario().seed });
    while (!p_simulation.finished() &&
           p_simulation.time() < robotik::Seconds(p_limit))
    {
        p_simulation.step(robotik::Seconds(0.01));
    }
    p_simulation.conclude();
    for (robotik::Check const& check : p_simulation.checks())
    {
        EXPECT_EQ(check.verdict, robotik::Check::Verdict::Passed)
            << check.text << ": " << check.detail;
    }
    return p_simulation.outcome();
}

TEST(GrafcetScenario, SequenceRetriesDetectionAfterTheCameraFailure)
{
    PickPlaceMission mission(true, true);
    compages::world::World world;
    robotik::Simulation simulation(
        world, robotik::loadScenario(repoFile(SEQUENCE), testAssets()).take(),
        nullptr, &mission);
    ASSERT_NE(simulation.cell().grafcet(), nullptr);
    EXPECT_EQ(runToTheEnd(simulation, 60.0),
              robotik::Simulation::Outcome::ControllerSucceeded);

    // The End step is active; Detect failed once (camera off), then passed.
    std::vector<std::string> const steps = simulation.cell().grafcet()->activeSteps();
    EXPECT_EQ(steps, std::vector<std::string>{ "PickAndPlace:X11" });
    std::size_t failed = 0U;
    for (robotik::SkillRun const& run : simulation.cell().skills().trace())
    {
        if (simulation.cell().skills().name(run.skill) == "Detect(red_cube)" &&
            run.state != robotik::SkillState::Succeeded)
        {
            ++failed;
        }
    }
    EXPECT_GE(failed, 1U);
}

TEST(GrafcetScenario, GemmaRunsOneCycleThenStopsOnAU)
{
    {
        PickPlaceMission mission(true, true);
        compages::world::World world;
        robotik::Simulation simulation(
            world, robotik::loadScenario(repoFile(GEMMA), testAssets()).take(),
            nullptr, &mission);
        EXPECT_EQ(runToTheEnd(simulation, 60.0),
                  robotik::Simulation::Outcome::ControllerSucceeded);
        // Back to A1 and to the waiting step.
        std::vector<std::string> const steps = simulation.cell().grafcet()->activeSteps();
        EXPECT_NE(std::ranges::find(steps, "Control:X0"), steps.end());
        EXPECT_NE(std::ranges::find(steps, "Production:X0"), steps.end());
    }
    {
        PickPlaceMission mission(true, true);
        compages::world::World world;
        robotik::Simulation simulation(
            world, robotik::loadScenario(repoFile(GEMMA_STOP), testAssets()).take(),
            nullptr, &mission);
        EXPECT_EQ(runToTheEnd(simulation, 30.0),
                  robotik::Simulation::Outcome::ControllerSucceeded);
        EXPECT_EQ(simulation.cell().grafcet()->operatorInputs().at("AU"), 1);
        EXPECT_NEAR(simulation.time().value(), 5.0, 0.1);
    }
}

TEST(GrafcetScenario, GemmaRearmsAndRestartsWithPushButtons)
{
    PickPlaceMission mission(true, true);
    compages::world::World world;
    robotik::Simulation simulation(
        world, robotik::loadScenario(repoFile(GEMMA_REARM), testAssets()).take(),
        nullptr, &mission);
    EXPECT_EQ(runToTheEnd(simulation, 60.0),
              robotik::Simulation::Outcome::ControllerSucceeded);
    // Push buttons are back to 0 after their cycle.
    std::map<std::string, int> const inputs = simulation.cell().grafcet()->operatorInputs();
    EXPECT_EQ(inputs.at("Dcy"), 0);
    EXPECT_EQ(inputs.at("Rearm"), 0);
    EXPECT_EQ(inputs.at("AU"), 0);
}

TEST(GrafcetScenario, PushButtonLastsOneCycleEvenWhenPaused)
{
    PickPlaceMission mission(true, true);
    compages::world::World world;
    robotik::Simulation simulation(
        world, robotik::loadScenario(repoFile(GEMMA_STOP), testAssets()).take(),
        nullptr, &mission);
    simulation.reset(robotik::Seed{ 42 });
    robotik::GrafcetController& grafcet = *simulation.cell().grafcet();
    // Pressed while no step runs: still 1 for the next cycle.
    ASSERT_TRUE(grafcet.pulse("Rearm"));
    EXPECT_EQ(grafcet.operatorInputs().at("Rearm"), 1);
    simulation.step(robotik::Seconds(0.01));
    EXPECT_EQ(grafcet.operatorInputs().at("Rearm"), 0);
    EXPECT_FALSE(grafcet.pulse("done(Home)")) << "not an operator input";
}
