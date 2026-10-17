// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

// The PDDL scenarios of the demos, end to end and headless.

#include "main.hpp"

#include "PainterMission.hpp"
#include "PickPlaceMission.hpp"

#include "Robotik/Runtime/RunReport.hpp"
#include "Robotik/Runtime/Simulation.hpp"

#include "Compages/World/World.hpp"

#include <sstream>

namespace
{

struct PlannedRun
{
    robotik::Simulation::Outcome outcome =
        robotik::Simulation::Outcome::Running;
    bool passed = false;
    double time = 0.0;
    std::vector<std::string> plans;
    std::vector<std::string> causes;
};

PlannedRun
run(robotik::Simulation& p_simulation, robotik::Seed p_seed, double p_limit)
{
    p_simulation.reset(p_seed);
    while (!p_simulation.finished() &&
           p_simulation.time() < robotik::Seconds(p_limit))
    {
        p_simulation.step(robotik::Seconds(0.01));
    }
    p_simulation.conclude();
    PlannedRun result;
    result.outcome = p_simulation.outcome();
    result.passed = p_simulation.passed();
    result.time = p_simulation.time().value();
    for (robotik::Check const& check : p_simulation.checks())
    {
        EXPECT_EQ(check.verdict, robotik::Check::Verdict::Passed)
            << check.text << ": " << check.detail << " at "
            << check.time.value() << " s";
    }
    robotik::PlanExecutor const* executor = p_simulation.planner();
    EXPECT_NE(executor, nullptr);
    if (executor != nullptr)
    {
        for (robotik::PlanningRecord const& record : executor->history())
        {
            std::string plan;
            for (robotik::GroundAction const& step : record.result.steps)
            {
                plan += step.toPddl();
            }
            result.plans.push_back(plan);
            result.causes.push_back(record.cause);
        }
    }
    return result;
}

robotik::Scenario scenario(char const* p_file)
{
    return robotik::Scenario::load(repoFile(p_file), testAssets());
}

} // namespace

TEST(PlannerScenario, PickAndPlacePlansFromTheGoalAndReplays)
{
    PickPlaceMission mission(true, true);
    compages::world::World world;
    robotik::Simulation simulation(
        world,
        scenario("plugins/PickAndPlace/scenarios/pick_and_place_pddl.scenario.yaml"),
        nullptr,
        &mission);
    ASSERT_NE(simulation.planner(), nullptr);
    EXPECT_EQ(simulation.planner()->objects().size(), 2u);

    PlannedRun const first = run(simulation, robotik::Seed{ 42 }, 60.0);
    EXPECT_EQ(first.outcome, robotik::Simulation::Outcome::ControllerSucceeded);
    EXPECT_TRUE(first.passed);
    ASSERT_EQ(first.plans.size(), 1u);
    EXPECT_EQ(first.causes[0], "start");
    EXPECT_NE(first.plans[0].find("(pick red_cube)"), std::string::npos)
        << first.plans[0];
    EXPECT_NE(first.plans[0].find("(place red_cube box)"), std::string::npos)
        << first.plans[0];
    EXPECT_EQ(simulation.planner()->done(),
              simulation.planner()->plan().size());

    // The generated tree runs the bound skills, then checks each step.
    std::string const& tree = simulation.planner()->history().front().tree;
    EXPECT_NE(tree.find("\"Grasp(red_cube)\""), std::string::npos) << tree;
    EXPECT_NE(tree.find("\"Check 2. pick(red_cube)\""), std::string::npos)
        << tree;

    PlannedRun const again = run(simulation, robotik::Seed{ 42 }, 60.0);
    EXPECT_EQ(first.plans, again.plans);
    EXPECT_DOUBLE_EQ(first.time, again.time);
}

TEST(PlannerScenario, PickAndPlaceReplansAroundACameraFailure)
{
    PickPlaceMission mission(true, true);
    compages::world::World world;
    robotik::Simulation simulation(
        world,
        scenario(
            "plugins/PickAndPlace/scenarios/pick_and_place_pddl_faults.scenario.yaml"),
        nullptr,
        &mission);

    PlannedRun const result = run(simulation, robotik::Seed{ 7 }, 60.0);
    EXPECT_EQ(result.outcome,
              robotik::Simulation::Outcome::ControllerSucceeded);
    EXPECT_TRUE(result.passed);
    ASSERT_GE(result.plans.size(), 2u);
    EXPECT_NE(result.causes[1].find("Detect(red_cube)"), std::string::npos)
        << result.causes[1];
    EXPECT_NE(result.causes[1].find("unavailable"), std::string::npos)
        << result.causes[1];
    EXPECT_GE(simulation.planner()->replans(), 1u);

    // --report: the plans and the replans are in the run record.
    robotik::RunRecord const record = robotik::RunRecord::capture(simulation, "default", {});
    EXPECT_TRUE(record.planner.used);
    EXPECT_EQ(record.planner.plans.size(), result.plans.size());
    EXPECT_EQ(record.planner.replans, simulation.planner()->replans());
    EXPECT_EQ(record.planner.plans[1].cause, result.causes[1]);
    ASSERT_FALSE(record.planner.plans.front().steps.empty());
    EXPECT_EQ(record.planner.plans.front().steps.front(), "detect(red_cube)");
    std::ostringstream json;
    robotik::RunReport{ "x", { record } }.writeJson(json);
    EXPECT_NE(json.str().find("\"replans\": "), std::string::npos) << json.str();
}

TEST(PlannerScenario, PainterLaysTheLayersInOrder)
{
    PainterMission mission(true);
    compages::world::World world;
    robotik::Simulation simulation(
        world,
        scenario("plugins/Painter/scenarios/painter_pddl.scenario.yaml"),
        nullptr,
        &mission);

    PlannedRun const result = run(simulation, robotik::Seed{ 7 }, 200.0);
    EXPECT_EQ(result.outcome,
              robotik::Simulation::Outcome::ControllerSucceeded);
    EXPECT_TRUE(result.passed);
    ASSERT_EQ(result.plans.size(), 1u);
    std::string const& plan = result.plans[0];
    std::size_t const red = plan.find("(mark red)");
    std::size_t const black = plan.find("(mark black)");
    std::size_t const blue = plan.find("(mark blue)");
    std::size_t const gold = plan.find("(mark gold)");
    ASSERT_NE(gold, std::string::npos) << plan;
    EXPECT_LT(red, black) << plan;
    EXPECT_LT(black, blue) << plan;
    EXPECT_LT(blue, gold) << plan;
    // Four dips, each followed by a rinse: the goal wants a clean brush.
    EXPECT_EQ(simulation.planner()->plan().size(), 12u) << plan;
}

TEST(PlannerScenario, ScenarioRefusesATreeAndAPlanner)
{
    robotik::ScenarioSource source = robotik::ScenarioSource::load(
        repoFile("plugins/PickAndPlace/scenarios/pick_and_place_pddl.scenario.yaml"));
    source.override("scenario.controller.behavior_tree",
                    robotik::ConfigNode("scenarios/pick_and_place.bt.yaml"));
    EXPECT_THROW((void)robotik::Scenario::read(source, testAssets()),
                 std::runtime_error);
}
