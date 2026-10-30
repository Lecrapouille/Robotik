// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

// Workflows: reading x.workflow.yaml, cycle time and bottleneck, measured
// durations.

#include "main.hpp"

#include "PickPlaceMission.hpp"

#include "Robotik/Package/Scenario/Workflow.hpp"
#include "Robotik/Package/Simulation.hpp"
#include "Robotik/Task/Workflow/WorkflowAnalysis.hpp"

#include "Compages/World/World.hpp"

#include <fstream>

/// Writes a workflow file in the test folder and reads it.
static robotik::Result<robotik::WorkflowDescription> read(std::string const& p_text)
{
    std::filesystem::path const file =
        std::filesystem::temp_directory_path() / "robotik_test.workflow.yaml";
    std::ofstream(file) << p_text;
    return robotik::loadWorkflow(file);
}

TEST(Workflow, CycleTimeAndBottleneckOfTheConveyor)
{
    robotik::WorkflowDescription const workflow =
        robotik::loadWorkflow(repoFile("plugins/DualArm/scenarios/conveyor.workflow.yaml"))
            .take();
    ASSERT_EQ(workflow.operators.size(), 3U);
    ASSERT_EQ(workflow.products.size(), 2U);
    EXPECT_EQ(workflow.products[1].count, 2U);

    robotik::WorkflowAnalysis const analysis = robotik::analyseWorkflow(workflow, 2U).take();
    // The right arm sorts one part B every 6 s: it sets the period.
    EXPECT_DOUBLE_EQ(analysis.cycle_time.value(), 6.0);
    EXPECT_DOUBLE_EQ(analysis.throughput, 2.0 / 6.0);
    EXPECT_EQ(analysis.critical,
              std::vector<std::string>{ "part_b: sort to bin_right (right)" });
    EXPECT_EQ(analysis.critical_operators, std::vector<std::string>{ "right" });
    // 4 operations, 2 cycles.
    ASSERT_EQ(analysis.bars.size(), 8U);
    EXPECT_DOUBLE_EQ(analysis.bars.front().start.value(), 0.0);
    EXPECT_NE(analysis.graph, nullptr);
    EXPECT_FALSE(analysis.critical_arcs.empty());
}

TEST(Workflow, RefusesUnknownOperatorsAndOtherOrders)
{
    auto const unknown = read("workflow:\n  id: w\n  operators: [ { id: a } ]\n"
                              "  products:\n    - id: p\n      route: [ { name: x, on: b, duration: 1 } ]\n");
    ASSERT_FALSE(unknown);
    EXPECT_NE(unknown.error().find("unknown operator 'b'"), std::string::npos) << unknown.error();

    auto const order = read("workflow:\n  id: w\n  operators: [ { id: a }, { id: b } ]\n"
                            "  products:\n    - id: p\n      route:\n"
                            "        - { name: x, on: b, duration: 1 }\n"
                            "        - { name: y, on: a, duration: 1 }\n");
    ASSERT_FALSE(order);
    EXPECT_NE(order.error().find("a flowshop"), std::string::npos) << order.error();

    auto const duration = read("workflow:\n  id: w\n  operators: [ { id: a } ]\n"
                               "  products:\n    - id: p\n      route: [ { name: x, on: a } ]\n");
    ASSERT_FALSE(duration);
    EXPECT_NE(duration.error().find("duration"), std::string::npos) << duration.error();
}

TEST(Workflow, MeasuredDurationsReplaceTheDeclaredOnes)
{
    PickPlaceMission mission(true, true);
    compages::world::World world;
    robotik::Simulation simulation(
        world,
        robotik::loadScenario(
            repoFile("plugins/PickAndPlace/scenarios/pick_and_place_grafcet.scenario.yaml"),
            testAssets())
            .take(),
        nullptr, &mission);
    simulation.reset(robotik::Seed{ 7 });
    while (!simulation.finished() && simulation.time() < robotik::Seconds(60.0))
    {
        simulation.step(robotik::Seconds(0.01));
    }
    std::map<std::string, robotik::Seconds> const measured =
        robotik::measuredDurations(simulation.cell().skills());
    ASSERT_TRUE(measured.contains("Detect(red_cube)"));

    robotik::WorkflowDescription workflow =
        robotik::loadWorkflow(repoFile("plugins/PickAndPlace/scenarios/pick_and_place.workflow.yaml"))
            .take();
    EXPECT_EQ(robotik::useMeasuredDurations(workflow, measured), 1U);
    EXPECT_DOUBLE_EQ(workflow.products[0].route[0].duration.value(),
                     measured.at("Detect(red_cube)").value());
}
