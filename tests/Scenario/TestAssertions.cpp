// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "main.hpp"

#include "PainterMission.hpp"

#include "Robotik/Package/Simulation.hpp"
#include "Robotik/Package/Scenario/Assertions.hpp"
#include "Robotik/Common/Config.hpp"

#include <map>

using Verdict = robotik::Check::Verdict;

/// Conditions are names of a table of values the test changes over time.
struct Board
{
    std::map<std::string, double, std::less<>> values;

    robotik::ConditionProbe probe()
    {
        return [this](std::string_view p_condition)
        {
            robotik::Metrics metrics;
            for (auto const& [name, value] : values)
            {
                metrics.set(name, value);
            }
            return robotik::evaluate(p_condition, metrics);
        };
    }
};

/// Parses one assertion from a YAML snippet.
static robotik::AssertionSpec spec(std::string const& p_yaml)
{
    return robotik::AssertionSpec::read(robotik::ConfigNode::parse(p_yaml).take());
}

TEST(ConfigNode, ReadsTypedValuesAndReportsBadOnes)
{
    robotik::ConfigNode const node = robotik::ConfigNode::parse(
        "a: 1.5\nb: [1, 2, 3]\nc: yes\nd: text\nr: [0.1, -0.1]\n"
        "p: { position: [1, 0, 0], rpy: [0, 0, 0] }\n").take();
    EXPECT_DOUBLE_EQ(node.number("a", 0.0), 1.5);
    EXPECT_DOUBLE_EQ(node.number("missing", 7.0), 7.0);
    EXPECT_DOUBLE_EQ(node.vector3("b", robotik::Vector3::ZERO).z, 3.0);
    EXPECT_TRUE(node.flag("c", false));
    EXPECT_EQ(node.text("d"), "text");
    EXPECT_DOUBLE_EQ(node.pose("p").position.x, 1.0);
    EXPECT_THROW((void)node.number("d", 0.0), std::runtime_error);
    EXPECT_THROW((void)node.vector3("a", robotik::Vector3::ZERO), std::runtime_error);
    EXPECT_THROW((void)node.range("r"), std::runtime_error);
}

TEST(ConfigNode, SetFollowsMapsAndSequenceIds)
{
    robotik::ConfigNode node =
        robotik::ConfigNode::parse("world:\n  entities:\n    - { id: a, mass: "
                                   "1 }\n    - { id: b, mass: 2 }\n").take();
    node.set("world.entities.b.mass", robotik::ConfigNode("5"));
    node.set("world.entities.0.color", robotik::ConfigNode::parse("[1, 0, 0]").take());
    node.set("world.floor.friction", robotik::ConfigNode("0.3"));
    robotik::ConfigNode const& entities = node.at("world").at("entities");
    EXPECT_DOUBLE_EQ(entities.item("b").number("mass", 0.0), 5.0);
    EXPECT_TRUE(entities.item("a").at("color").isSequence());
    EXPECT_DOUBLE_EQ(node.at("world").at("floor").number("friction", 0.0), 0.3);
    EXPECT_THROW(node.set("world.entities.c.mass", robotik::ConfigNode("1")),
                 std::runtime_error);
}

TEST(Assertions, ShorthandIsJudgedAtTheEnd)
{
    Board board{ { { "x", 0.0 } } };
    robotik::AssertionEngine engine({ spec("x > 1") });
    engine.update(robotik::Seconds(1.0), board.probe());
    EXPECT_EQ(engine.checks()[0].verdict, Verdict::Pending);
    EXPECT_FALSE(engine.checks()[0].ok());
    board.values["x"] = 2.0;
    engine.update(robotik::Seconds(2.0), board.probe());
    EXPECT_TRUE(engine.checks()[0].ok());
    engine.conclude(robotik::Seconds(3.0), board.probe());
    EXPECT_EQ(engine.checks()[0].verdict, Verdict::Passed);
    EXPECT_TRUE(engine.passed());
}

TEST(Assertions, AlwaysKeepsTheFirstBreach)
{
    Board board{ { { "gap", 0.001 } } };
    robotik::AssertionEngine engine(
        { spec("{ condition: gap < 0.002, phase: always }") });
    engine.update(robotik::Seconds(1.0), board.probe());
    board.values["gap"] = 0.004;
    engine.update(robotik::Seconds(2.0), board.probe());
    board.values["gap"] = 0.0;
    engine.update(robotik::Seconds(3.0), board.probe());
    engine.conclude(robotik::Seconds(4.0), board.probe());
    robotik::Check const& check = engine.checks()[0];
    EXPECT_EQ(check.verdict, Verdict::Failed);
    EXPECT_DOUBLE_EQ(check.value, 0.004);
    EXPECT_DOUBLE_EQ(check.time.value(), 2.0);
    EXPECT_EQ(check.expected, "< 0.002");
    EXPECT_FALSE(engine.stopRequested());
}

TEST(Assertions, StopOnFailureIsRequested)
{
    Board board{ { { "ok", 0.0 } } };
    robotik::AssertionEngine engine(
        { spec("{ condition: ok, phase: always, stop: true }") });
    engine.update(robotik::Seconds(0.1), board.probe());
    EXPECT_TRUE(engine.stopRequested());
}

TEST(Assertions, WithinAtAndEventually)
{
    Board board{ { { "x", 0.0 } } };
    robotik::AssertionEngine engine(
        { spec("{ condition: x, within: 2.0 }"),
          spec("{ condition: x, at: 5.0 }"),
          spec("{ condition: x, eventually: true }") });
    engine.update(robotik::Seconds(1.0), board.probe());
    for (robotik::Check const& check : engine.checks())
    {
        EXPECT_EQ(check.verdict, Verdict::Pending);
    }
    engine.update(robotik::Seconds(2.5), board.probe());
    EXPECT_EQ(engine.checks()[0].verdict, Verdict::Failed);
    board.values["x"] = 1.0;
    engine.update(robotik::Seconds(4.0), board.probe());
    EXPECT_EQ(engine.checks()[1].verdict, Verdict::Pending);
    EXPECT_EQ(engine.checks()[2].verdict, Verdict::Passed);
    engine.update(robotik::Seconds(5.0), board.probe());
    EXPECT_EQ(engine.checks()[1].verdict, Verdict::Passed);
}

TEST(Assertions, AtFailsWhenTheRunEndsBefore)
{
    Board board{ { { "x", 1.0 } } };
    robotik::AssertionEngine engine({ spec("{ condition: x, at: 5.0 }") });
    engine.conclude(robotik::Seconds(3.0), board.probe());
    EXPECT_EQ(engine.checks()[0].verdict, Verdict::Failed);
}

TEST(Assertions, SequenceKeepsTheOrder)
{
    Board board{ { { "a", 0.0 }, { "b", 0.0 } } };
    robotik::AssertionEngine good({ spec("{ sequence: [a, b] }") });
    robotik::AssertionEngine bad({ spec("{ sequence: [a, b] }") });
    good.update(robotik::Seconds(0.1), board.probe());
    board.values["a"] = 1.0;
    good.update(robotik::Seconds(0.2), board.probe());
    board.values["b"] = 1.0;
    good.update(robotik::Seconds(0.3), board.probe());
    EXPECT_EQ(good.checks()[0].verdict, Verdict::Passed);

    board.values = { { "a", 0.0 }, { "b", 1.0 } };
    bad.update(robotik::Seconds(0.1), board.probe());
    EXPECT_EQ(bad.checks()[0].verdict, Verdict::Failed);
    EXPECT_NE(bad.checks()[0].detail.find("'b' before 'a'"), std::string::npos);
}

TEST(Assertions, UnknownValueFails)
{
    Board board;
    robotik::AssertionEngine engine({ spec("nothing > 1") });
    engine.update(robotik::Seconds(0.1), board.probe());
    EXPECT_EQ(engine.checks()[0].verdict, Verdict::Failed);
    EXPECT_NE(engine.checks()[0].detail.find("nothing"), std::string::npos);
}

TEST(Assertions, InvalidSpecsAreRefused)
{
    EXPECT_THROW((void)spec("{ phase: always }"), std::runtime_error);
    EXPECT_THROW((void)spec("{ condition: x, phase: sometimes }"),
                 std::runtime_error);
    EXPECT_THROW((void)spec("{ sequence: [a] }"), std::runtime_error);
}

TEST(Simulation, AMisplacedCanvasBreaksThePainterAssertions)
{
    // Same world, the canvas hangs 1 m behind the easel, 35 cm above the
    // floor: it falls far from everything. Zero robot collision, yet the
    // scene is wrong.
    robotik::ScenarioSource source = robotik::ScenarioSource::load(
        repoFile("plugins/Painter/scenarios/painter.scenario.yaml")).take();
    source.override("world.entities.canvas.attach.parent",
                    robotik::ConfigNode("easel"));
    source.override("world.entities.canvas.attach.transform.position",
                    robotik::ConfigNode::parse("[-1.0, 0.0, 0.6]").take());
    PainterMission mission;
    compages::world::World world;
    robotik::Simulation simulation(
        world,
        robotik::Scenario::read(source, testAssets()).take(),
        nullptr,
        &mission);
    simulation.cell().suspend(true);
    for (int i = 0; i < 200; ++i)
    {
        simulation.step(robotik::Seconds(0.01));
    }
    EXPECT_EQ(simulation.cell().query("on(canvas, easel)"), 0.0);
    EXPECT_EQ(simulation.cell().query("touching(canvas, easel)"), 0.0);
    EXPECT_GT(*simulation.cell().query("pose_error(canvas)"), 0.05);
    EXPECT_EQ(simulation.cell().query("collision_count(robot)"), 0.0);
    EXPECT_THROW((void)simulation.cell().query("pose_error(nobody)"),
                 std::invalid_argument);
    EXPECT_FALSE(simulation.evaluate("pose_error(nobody) < 1").known);
}
