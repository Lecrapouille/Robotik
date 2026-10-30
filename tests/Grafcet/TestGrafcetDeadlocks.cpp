// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

// Deadlocks of a GRAFCET and the resources of its skills, found before the
// run.

#include "main.hpp"

#include "Robotik/Task/Grafcet/GrafcetDeadlocks.hpp"
#include "Robotik/Task/Workflow/WorkflowGrafcet.hpp"
#include "Robotik/Package/Scenario/Workflow.hpp"

#include "PetriNet/PetriNet.hpp"

/// X0 -> X1 (action Run) and X2 (action Use); X2 -> X3 when Use is done;
/// X1 . X3 -> X4 (End). Run holds its resources while X1 is active.
static std::vector<tpne::Net> branches()
{
    tpne::Net graph(tpne::TypeOfNet::GRAFCET);
    graph.name = "G";
    for (char const* caption : { "start", "run", "use", "used", "End" })
    {
        (void)graph.addPlace(graph.places().empty() ? 1U : 0U, caption);
    }
    tpne::Action run;
    run.name = "Run";
    graph.places()[1].actions.push_back(run);
    tpne::Action use;
    use.name = "Use";
    graph.places()[2].actions.push_back(use);
    auto const link = [&graph](std::vector<std::size_t> p_in, std::string const& p_code,
                               std::vector<std::size_t> p_out)
    {
        std::size_t const t = graph.addTransition(p_code).id;
        for (std::size_t const step : p_in)
        {
            (void)graph.addArc(graph.places()[step], graph.transitions()[t]);
        }
        for (std::size_t const step : p_out)
        {
            (void)graph.addArc(graph.transitions()[t], graph.places()[step]);
        }
    };
    link({ 0U }, "1", { 1U, 2U });
    link({ 2U }, "done(Use)", { 3U });
    link({ 1U, 3U }, "1", { 4U });
    return { graph };
}

TEST(GrafcetDeadlocks, AStepHoldingABeltBlocksTheOtherBranch)
{
    // Run and Use both need the belt: X1 holds it until X3, which needs Use.
    robotik::GrafcetDeadlocks const found =
        robotik::findDeadlocks(branches(), { { "Run", { "belt" } }, { "Use", { "belt" } } }, "X4");
    EXPECT_TRUE(found.complete);
    ASSERT_EQ(found.deadlocks.size(), 1U);
    EXPECT_NE(found.deadlocks[0].find("G:X1 (run) holds belt"), std::string::npos)
        << found.deadlocks[0];
    EXPECT_NE(found.deadlocks[0].find("G:X2 (use) waits for belt"), std::string::npos)
        << found.deadlocks[0];
    ASSERT_NE(found.net, nullptr);
    // 5 steps, 2 places "holds", the belt.
    EXPECT_EQ(found.net->places().size(), 8U);
}

TEST(GrafcetDeadlocks, NoDeadlockWithoutASharedResource)
{
    robotik::GrafcetDeadlocks const found =
        robotik::findDeadlocks(branches(), { { "Run", { "belt" } }, { "Use", { "arm" } } }, "X4");
    EXPECT_TRUE(found.deadlocks.empty());
    // Without the success, the end is a marking of last steps only.
    EXPECT_TRUE(robotik::findDeadlocks(branches(), { { "Use", { "arm" } } }, "").deadlocks.empty());
}

TEST(GrafcetDeadlocks, TheGeneratedFactoryHasNone)
{
    robotik::WorkflowDescription const workflow =
        robotik::loadWorkflow(repoFile("plugins/DualArm/scenarios/factory.workflow.yaml")).take();
    std::vector<tpne::Net> graphs;
    std::string const end = robotik::workflowGrafcet(workflow, graphs).take();
    std::map<std::string, std::vector<std::string>> resources;
    for (tpne::Place const& place : graphs.front().places())
    {
        for (tpne::Action const& action : place.actions)
        {
            resources[action.name] = { action.name.substr(0U, 2U) + "_arm" };
        }
    }
    robotik::GrafcetDeadlocks const found = robotik::findDeadlocks(graphs, resources, end);
    EXPECT_TRUE(found.complete);
    EXPECT_GT(found.states, 1000U);
    EXPECT_TRUE(found.deadlocks.empty()) << found.deadlocks.front();
}
