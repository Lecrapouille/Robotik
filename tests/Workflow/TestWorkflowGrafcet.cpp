// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

// The GRAFCET generated from a workflow: recipes, buffers, parts; what the
// generator refuses; the scenario block grafcet: { workflow: ... }.

#include "main.hpp"

#include "Robotik/Package/Scenario/Scenario.hpp"
#include "Robotik/Package/Scenario/Workflow.hpp"
#include "Robotik/Task/Workflow/WorkflowAnalysis.hpp"
#include "Robotik/Task/Workflow/WorkflowGrafcet.hpp"

#include "IO/Exports/Exports.hpp"
#include "IO/Imports/Imports.hpp"
#include "PetriNet/PetriNet.hpp"

#include <algorithm>
#include <fstream>

static char const* const FACTORY = "plugins/DualArm/scenarios/factory.workflow.yaml";

/// Writes a workflow file in the test folder and reads it.
static robotik::Result<robotik::WorkflowDescription> read(std::string const& p_text)
{
    std::filesystem::path const file =
        std::filesystem::temp_directory_path() / "robotik_grafcet.workflow.yaml";
    std::ofstream(file) << p_text;
    return robotik::loadWorkflow(file);
}

/// Error of the generator for a workflow, or empty.
static std::string generationError(robotik::WorkflowDescription const& p_workflow)
{
    std::vector<tpne::Net> graphs;
    auto const generated = robotik::workflowGrafcet(p_workflow, graphs);
    return generated ? std::string() : generated.error();
}

/// Two machines, one part, a buffer between them.
static std::string const LINE = "workflow:\n"
                                "  id: line\n"
                                "  operators:\n"
                                "    - { id: a, take: [ \"a.Pick({part})\" ], give: [ \"a.Place({to})\" ] }\n"
                                "    - { id: b, take: [ \"b.Pick({part})\" ], give: [ \"b.Place({to})\" ] }\n"
                                "  buffers: [ { id: tray, after: a, places: { p: spot } } ]\n"
                                "  output: bin\n"
                                "  products:\n"
                                "    - id: p\n"
                                "      parts: [p1]\n"
                                "      route:\n"
                                "        - { name: cut, on: a, duration: 1 }\n"
                                "        - { name: drill, on: b, duration: 0.5 }\n";

TEST(WorkflowGrafcet, ReadsRecipesBuffersAndParts)
{
    robotik::WorkflowDescription const workflow =
        robotik::loadWorkflow(repoFile(FACTORY)).take();
    ASSERT_EQ(workflow.operators.size(), 2U);
    EXPECT_EQ(workflow.operators[0].take.size(), 4U);
    EXPECT_EQ(workflow.operators[0].give.size(), 3U);
    EXPECT_EQ(workflow.operators[1].take.back(), "M2.Over({to})");
    ASSERT_EQ(workflow.buffers.size(), 1U);
    EXPECT_EQ(workflow.buffers[0].after, "M1");
    EXPECT_TRUE(workflow.buffers[0].zone);
    ASSERT_EQ(workflow.buffers[0].places.size(), 3U);
    EXPECT_EQ(workflow.buffers[0].places[1].second, "slot2");
    EXPECT_EQ(workflow.output, "output");
    EXPECT_EQ(workflow.products[2].parts, (std::vector<std::string>{ "p3a", "p3b" }));

    // M1 is the bottleneck of the factory: the simulator draws it in red.
    robotik::WorkflowAnalysis const analysis = robotik::analyseWorkflow(workflow).take();
    EXPECT_EQ(analysis.critical_operators, std::vector<std::string>{ "M1" });
}

TEST(WorkflowGrafcet, GeneratesTheFactory)
{
    robotik::WorkflowDescription const workflow =
        robotik::loadWorkflow(repoFile(FACTORY)).take();
    std::vector<tpne::Net> graphs;
    std::string const end = robotik::workflowGrafcet(workflow, graphs).take();
    ASSERT_EQ(graphs.size(), 1U);
    tpne::Net const& net = graphs.front();
    EXPECT_EQ(net.type(), tpne::TypeOfNet::GRAFCET);
    EXPECT_EQ(net.name, "Production");
    // Per part: M1 4 take, machining, wait, 3 give, idle, part done (11);
    // M2 wait, 5 take, machining, give, idle (9). Six parts, the station
    // (zone and 3 places), the two first idle steps, End.
    EXPECT_EQ(net.places().size(), 6U * 11U + 6U * 9U + 6U + 1U);
    EXPECT_EQ(net.transitions().size(), 6U * 10U + 6U * 9U + 1U);
    EXPECT_EQ(end, "Production:X" + std::to_string(net.places().size() - 1U));
    EXPECT_EQ(net.places().back().caption, "End");

    auto const hasTransition = [&net](std::string const& p_code)
    {
        return std::ranges::any_of(net.transitions(), [&p_code](tpne::Transition const& p_t)
                                   { return p_t.caption == p_code; });
    };
    EXPECT_TRUE(hasTransition("done(M1.Grasp(p1a)) . holding(M1, p1a)"));
    EXPECT_TRUE(hasTransition("done(M2.Over(output))"));
    // M1 machines P1 for 2 s, then waits for the station: M2 goes first.
    auto const machining = std::ranges::find(net.places(), std::string("M1 machining P1 (2 s)"),
                                             &tpne::Place::caption);
    ASSERT_NE(machining, net.places().end());
    EXPECT_TRUE(hasTransition("2s/X" + std::to_string(machining->id)));
    auto const guards = std::ranges::count_if(net.transitions(), [](tpne::Transition const& p_t)
                                              { return p_t.caption.starts_with("/(X"); });
    EXPECT_EQ(guards, 6);
    // Each step of a recipe requests its skill.
    auto const grasp = std::ranges::find_if(net.places(), [](tpne::Place const& p_place)
                                            { return p_place.caption == "M2.Grasp(p3b)"; });
    ASSERT_NE(grasp, net.places().end());
    ASSERT_EQ(grasp->actions.size(), 1U);
    EXPECT_EQ(grasp->actions[0].name, "M2.Grasp(p3b)");

    // TimedPetriNetEditor reads it back as it was made.
    std::filesystem::path const file =
        std::filesystem::temp_directory_path() / "robotik_factory.grafcet.json";
    ASSERT_EQ(tpne::exportAllNetsToJSON(graphs, file.string()), "");
    std::vector<tpne::Net> read;
    ASSERT_EQ(tpne::importAllNetsFromJSON(read, file.string()), "");
    ASSERT_EQ(read.size(), 1U);
    EXPECT_EQ(read[0].places().size(), net.places().size());
    EXPECT_EQ(read[0].transitions().size(), net.transitions().size());
    EXPECT_EQ(read[0].tokens(), net.tokens());
    EXPECT_EQ(read[0].places()[10].actions.size(), net.places()[10].actions.size());
}

TEST(WorkflowGrafcet, RunsTheSkillOfAnOperationAndSharesAPlace)
{
    robotik::WorkflowDescription const workflow =
        robotik::loadWorkflow(repoFile("plugins/DualArm/scenarios/factory_belt.workflow.yaml"))
            .take();
    ASSERT_EQ(workflow.buffers.size(), 2U);
    EXPECT_EQ(workflow.buffers[0].place, "pick");
    EXPECT_TRUE(workflow.buffers[0].places.empty());

    std::vector<tpne::Net> graphs;
    (void)robotik::workflowGrafcet(workflow, graphs).take();
    tpne::Net const& net = graphs.front();
    // One step "pick free" for the three kinds.
    auto const free = std::ranges::count_if(net.places(), [](tpne::Place const& p_place)
                                            { return p_place.caption == "pick free"; });
    EXPECT_EQ(free, 1);
    // The belt runs 0.55 s for each part: the action of the step of the
    // operation, ended by a time receptivity.
    auto const bring = std::ranges::find(net.places(), std::string("feeder bring P2 (0.55 s)"),
                                         &tpne::Place::caption);
    ASSERT_NE(bring, net.places().end());
    ASSERT_EQ(bring->actions.size(), 1U);
    EXPECT_EQ(bring->actions[0].name, "feeder_belt.Run");
    EXPECT_TRUE(std::ranges::any_of(net.transitions(), [&bring](tpne::Transition const& p_t)
                                    { return p_t.caption == "0.55s/X" + std::to_string(bring->id); }));

    // Without a duration the step waits for the skill.
    robotik::WorkflowDescription untimed = workflow;
    for (robotik::WorkflowProduct& product : untimed.products)
    {
        product.route[0].duration = robotik::Seconds(0.0);
    }
    (void)robotik::workflowGrafcet(untimed, graphs).take();
    EXPECT_TRUE(std::ranges::any_of(graphs.front().transitions(), [](tpne::Transition const& p_t)
                                    { return p_t.caption == "done(feeder_belt.Run)"; }));

    auto const both = read(LINE + "  buffers: [ { id: tray, after: a, place: s, places: { p: x } } ]\n");
    ASSERT_FALSE(both);
}

TEST(WorkflowGrafcet, StartedOperationsMoveTheTokensOfTheEventGraph)
{
    robotik::WorkflowDescription const workflow =
        robotik::loadWorkflow(repoFile(FACTORY)).take();
    std::vector<tpne::Net> graphs;
    (void)robotik::workflowGrafcet(workflow, graphs).take();
    // At the start nothing started.
    EXPECT_TRUE(robotik::startedOperations(workflow, graphs).empty());

    // M1 machines its second part (p2a), M2 still waits for its first one.
    tpne::Net& production = graphs.front();
    for (tpne::Place& place : production.places())
    {
        place.tokens = 0U;
    }
    auto const step = [&production](std::string const& p_caption)
    {
        return std::ranges::find(production.places(), p_caption, &tpne::Place::caption);
    };
    step("M1 machining P2 (3 s)")->tokens = 1U;
    step("M2 idle")->tokens = 1U;
    std::map<std::string, std::size_t> const started = robotik::startedOperations(workflow, graphs);
    EXPECT_EQ(started, (std::map<std::string, std::size_t>{ { "M1_P1", 1U }, { "M1_P2", 1U } }));

    // The tokens move, none is lost (each transition takes and gives two).
    robotik::WorkflowAnalysis const analysis = robotik::analyseWorkflow(workflow).take();
    std::vector<std::size_t> const before = analysis.graph->tokens();
    std::vector<std::size_t> const after = robotik::eventGraphMarking(*analysis.graph, started);
    ASSERT_EQ(after.size(), before.size());
    EXPECT_NE(after, before);
    std::size_t total_before = 0U;
    std::size_t total_after = 0U;
    for (std::size_t i = 0U; i < before.size(); ++i)
    {
        total_before += before[i];
        total_after += after[i];
    }
    EXPECT_EQ(total_after, total_before);
    EXPECT_EQ(robotik::eventGraphMarking(*analysis.graph, {}), before);
}

TEST(WorkflowGrafcet, GeneratesTheGemmaOfTheCell)
{
    robotik::WorkflowDescription workflow = read(LINE).take();
    workflow.gemma = "default";
    std::vector<tpne::Net> graphs;
    std::string const end = robotik::workflowGrafcet(workflow, graphs).take();
    ASSERT_EQ(graphs.size(), 3U);
    EXPECT_EQ(graphs[0].name, "Safety");
    EXPECT_EQ(graphs[1].name, "Control");
    EXPECT_EQ(graphs[2].name, "Production");
    EXPECT_TRUE(end.starts_with("Production:X"));

    // The part starts in F1 only; A2 goes back to A1 when a and b are idle.
    auto const has = [](tpne::Net const& p_net, auto p_test)
    { return std::ranges::any_of(p_net.transitions(), p_test); };
    EXPECT_TRUE(has(graphs[2], [](tpne::Transition const& p_t)
                    { return p_t.caption.starts_with("Control:X"); }));
    EXPECT_FALSE(has(graphs[1], [](tpne::Transition const& p_t)
                     { return p_t.caption == "Production:X0"; }));
    EXPECT_TRUE(has(graphs[1], [](tpne::Transition const& p_t)
                    { return p_t.caption.starts_with("(Production:X") &&
                             p_t.caption.find(") . (Production:X") != std::string::npos; }));

    auto const unknown = read(LINE + "  gemma: custom\n");
    ASSERT_FALSE(unknown);
    EXPECT_NE(unknown.error().find("gemma: 'custom'"), std::string::npos) << unknown.error();
}

TEST(WorkflowGrafcet, RefusesWhatCannotBeGenerated)
{
    robotik::WorkflowDescription const line = read(LINE).take();
    EXPECT_EQ(generationError(line), "");

    robotik::WorkflowDescription twice = line;
    twice.operators[1].capacity = 2U;
    EXPECT_NE(generationError(twice).find("one operator per unit"), std::string::npos);

    robotik::WorkflowDescription unbuffered = line;
    unbuffered.buffers.clear();
    EXPECT_NE(generationError(unbuffered).find("declare a buffer after a"), std::string::npos);

    robotik::WorkflowDescription placeless = line;
    placeless.buffers[0].places.clear();
    EXPECT_NE(generationError(placeless).find("no place for p"), std::string::npos);

    robotik::WorkflowDescription nowhere = line;
    nowhere.output.clear();
    EXPECT_NE(generationError(nowhere).find("set workflow.output"), std::string::npos);

    robotik::WorkflowDescription empty = line;
    empty.products[0].parts.clear();
    EXPECT_NE(generationError(empty).find("nothing to make"), std::string::npos);

    std::string placeholder = LINE;
    placeholder.replace(placeholder.find("{to}"), 4U, "{xx}");
    auto const wrong = read(placeholder);
    ASSERT_FALSE(wrong);
    EXPECT_NE(wrong.error().find("unknown placeholder '{xx}'"), std::string::npos) << wrong.error();
}

TEST(WorkflowGrafcet, ScenarioReadsTheWorkflowBlock)
{
    robotik::Scenario const scenario =
        robotik::loadScenario(repoFile("plugins/DualArm/scenarios/factory.scenario.yaml"),
                              testAssets())
            .take();
    ASSERT_TRUE(scenario.controller.grafcet.has_value());
    robotik::GrafcetSpec const& spec = *scenario.controller.grafcet;
    EXPECT_EQ(spec.file.filename(), "factory.workflow.yaml");
    ASSERT_TRUE(spec.workflow.has_value());
    EXPECT_EQ(spec.workflow->id, "factory_line");
    EXPECT_TRUE(spec.success.empty());
}
