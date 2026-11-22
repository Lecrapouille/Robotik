// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "main.hpp"

#include "PainterLogic.hpp"

#include "RobotIK/Package/Simulation.hpp"
#include "RobotIK/Package/Scenario/Assertions.hpp"
#include "RobotIK/Common/Config.hpp"

#include <map>

using Verdict = robotik::Check::Verdict;

/// Conditions are names of a table of values the test changes over time.
struct Board
{
    std::map<std::string, double, std::less<>> values;

    robotik::ConditionProbe probe(robotik::AssertionEngine const& p_engine)
    {
        return [this, &p_engine](std::size_t p_condition)
        {
            robotik::Metrics metrics;
            for (auto const& [name, value] : values)
            {
                metrics.set(name, value);
            }
            return robotik::evaluate(p_engine.conditions()[p_condition], metrics);
        };
    }
};

/// Parses one assertion from a YAML snippet.
static robotik::Result<robotik::AssertionSpec> read(std::string const& p_yaml)
{
    return robotik::readAssertion(robotik::ConfigNode::parse(p_yaml).take());
}

/// Reads an assertion that is valid.
static robotik::AssertionSpec spec(std::string const& p_yaml)
{
    return read(p_yaml).take();
}

TEST(ConfigNode, ReadsTypedValuesAndReportsBadOnes)
{
    robotik::ConfigNode const node = robotik::ConfigNode::parse(
        "a: 1.5\nb: [1, 2, 3]\nc: yes\nd: text\nr: [0.1, -0.1]\n"
        "p: { position: [1, 0, 0], rpy: [0, 0, 0] }\n").take();
    EXPECT_DOUBLE_EQ(node.number("a", 0.0).value(), 1.5);
    EXPECT_DOUBLE_EQ(node.number("missing", 7.0).value(), 7.0);
    EXPECT_DOUBLE_EQ(node.vector3("b", robotik::Vector3::ZERO).value().z, 3.0);
    EXPECT_TRUE(node.flag("c", false).value());
    EXPECT_EQ(node.text("d"), "text");
    EXPECT_DOUBLE_EQ(node.pose("p").value().position.x, 1.0);

    // A file is user input: a bad value is a failed Result, not an exception.
    robotik::Result<double> const word = node.number("d", 0.0);
    ASSERT_FALSE(word);
    EXPECT_EQ(word.error(), "'d' is not a number: 'text'");
    EXPECT_FALSE(node.vector3("a", robotik::Vector3::ZERO));
    EXPECT_FALSE(node.range("r"));
    EXPECT_FALSE(node.flag("d", false));

    // Code that may throw (a plugin, a test) takes the value with require.
    EXPECT_DOUBLE_EQ(robotik::require(node.number("a", 0.0)), 1.5);
    EXPECT_THROW((void)robotik::require(node.number("d", 0.0)),
                 std::runtime_error);
}

TEST(ConfigNode, SetFollowsMapsAndSequenceIds)
{
    robotik::ConfigNode node =
        robotik::ConfigNode::parse("world:\n  entities:\n    - { id: a, mass: "
                                   "1 }\n    - { id: b, mass: 2 }\n").take();
    EXPECT_TRUE(node.set("world.entities.b.mass", robotik::ConfigNode("5")));
    EXPECT_TRUE(node.set("world.entities.0.color", robotik::ConfigNode::parse("[1, 0, 0]").take()));
    EXPECT_TRUE(node.set("world.floor.friction", robotik::ConfigNode("0.3")));
    robotik::ConfigNode const& entities = node.at("world").at("entities");
    EXPECT_DOUBLE_EQ(entities.item("b").number("mass", 0.0).value(), 5.0);
    EXPECT_TRUE(entities.item("a").at("color").isSequence());
    EXPECT_DOUBLE_EQ(
        node.at("world").at("floor").number("friction", 0.0).value(), 0.3);
    EXPECT_FALSE(node.set("world.entities.c.mass", robotik::ConfigNode("1")));
}

TEST(ConfigNode, UnknownKeyIsRefusedWithTheClosestKey)
{
    robotik::ConfigNode const node =
        robotik::ConfigNode::parse("condition: time\nphse: always\n").take();
    robotik::Result<void> const typo =
        node.checkKeys({ "condition", "phase" }, "assertion");
    ASSERT_FALSE(typo) << "a typo must be refused";
    EXPECT_EQ(typo.error(),
              "assertion: unknown key 'phse' (did you mean 'phase'?)");

    robotik::ConfigNode const far =
        robotik::ConfigNode::parse("colour: red\n").take();
    robotik::Result<void> const unknown =
        far.checkKeys({ "condition", "phase" }, "assertion");
    ASSERT_FALSE(unknown) << "an unknown key must be refused";
    EXPECT_EQ(unknown.error(),
              "assertion: unknown key 'colour' (known keys: condition phase)");

    EXPECT_TRUE(node.checkKeys({ "condition", "phse" }, "assertion"));
    EXPECT_TRUE(robotik::ConfigNode().checkKeys({}, "nothing"));
}

TEST(Assertions, TypoInAKeyIsRefused)
{
    robotik::Result<robotik::AssertionSpec> const typo =
        read("{ condition: \"time > 1\", phse: always }");
    ASSERT_FALSE(typo);
    EXPECT_NE(typo.error().find("did you mean 'phase'"), std::string::npos)
        << typo.error();
}

TEST(Assertions, ShorthandIsJudgedAtTheEnd)
{
    Board board{ { { "x", 0.0 } } };
    robotik::AssertionEngine engine({ spec("(> x 1)") });
    engine.update(robotik::Seconds(1.0), board.probe(engine));
    EXPECT_EQ(engine.checks()[0].verdict, Verdict::Pending);
    EXPECT_FALSE(robotik::ok(engine.checks()[0]));
    board.values["x"] = 2.0;
    engine.update(robotik::Seconds(2.0), board.probe(engine));
    EXPECT_TRUE(robotik::ok(engine.checks()[0]));
    engine.conclude(robotik::Seconds(3.0), board.probe(engine));
    EXPECT_EQ(engine.checks()[0].verdict, Verdict::Passed);
    EXPECT_TRUE(engine.passed());
}

TEST(Assertions, AlwaysKeepsTheFirstBreach)
{
    Board board{ { { "gap", 0.001 } } };
    robotik::AssertionEngine engine(
        { spec("{ condition: (< gap 0.002), phase: always }") });
    engine.update(robotik::Seconds(1.0), board.probe(engine));
    board.values["gap"] = 0.004;
    engine.update(robotik::Seconds(2.0), board.probe(engine));
    board.values["gap"] = 0.0;
    engine.update(robotik::Seconds(3.0), board.probe(engine));
    engine.conclude(robotik::Seconds(4.0), board.probe(engine));
    robotik::Check const& check = engine.checks()[0];
    EXPECT_EQ(check.verdict, Verdict::Failed);
    EXPECT_DOUBLE_EQ(check.value, 0.004);
    EXPECT_DOUBLE_EQ(check.time.value(), 2.0);
    EXPECT_EQ(check.expected, "< 0.002");
    EXPECT_FALSE(engine.stopRequested());
}

TEST(Assertions, ToleranceForgivesAShortBreachOnly)
{
    Board board{ { { "gap", 0.001 } } };
    robotik::AssertionEngine engine(
        { spec("{ condition: (< gap 0.002), phase: always, for: 0.5 }") });
    auto at = [&](double p_time, double p_gap)
    {
        board.values["gap"] = p_gap;
        engine.update(robotik::Seconds(p_time), board.probe(engine));
    };
    at(1.0, 0.004); // a transient of 0.2 s
    at(1.2, 0.004);
    at(1.4, 0.001);
    EXPECT_EQ(engine.checks()[0].verdict, Verdict::Pending);
    at(2.0, 0.004); // false for 0.6 s: fails, dated at its start
    at(2.3, 0.004);
    at(2.6, 0.004);
    robotik::Check const& check = engine.checks()[0];
    EXPECT_EQ(check.verdict, Verdict::Failed);
    EXPECT_DOUBLE_EQ(check.time.value(), 2.0);
    EXPECT_NE(check.detail.find("false for more than"), std::string::npos);

    // A breach shorter than the tolerance at the end passes.
    robotik::AssertionEngine late(
        { spec("{ condition: (< gap 0.002), phase: always, for: 0.5 }") });
    board.values["gap"] = 0.001;
    late.update(robotik::Seconds(1.0), board.probe(late));
    board.values["gap"] = 0.004;
    late.conclude(robotik::Seconds(1.1), board.probe(late));
    EXPECT_EQ(late.checks()[0].verdict, Verdict::Passed);

    EXPECT_FALSE(read("{ condition: (< gap 1), for: 0.5 }"));
    EXPECT_FALSE(read("{ condition: (< gap 1), phase: always, for: -1 }"));
}

TEST(Assertions, UnknownSkipIgnoresTheStepsWithoutAnswer)
{
    Board board;
    robotik::AssertionEngine engine(
        { spec("{ condition: (< gap 0.002), phase: always, unknown: skip }") });
    engine.update(robotik::Seconds(1.0), board.probe(engine)); // no gap yet
    board.values["gap"] = 0.001;
    engine.update(robotik::Seconds(2.0), board.probe(engine));
    board.values.clear();
    engine.update(robotik::Seconds(3.0), board.probe(engine));
    EXPECT_EQ(engine.checks()[0].verdict, Verdict::Pending);
    engine.conclude(robotik::Seconds(4.0), board.probe(engine));
    EXPECT_EQ(engine.checks()[0].verdict, Verdict::Passed);

    // Never answered: still a failure.
    robotik::AssertionEngine never(
        { spec("{ condition: (< gap 0.002), phase: always, unknown: skip }") });
    never.update(robotik::Seconds(1.0), board.probe(never));
    never.conclude(robotik::Seconds(2.0), board.probe(never));
    EXPECT_EQ(never.checks()[0].verdict, Verdict::Failed);

    // Without skip, an unknown step fails as before.
    robotik::AssertionEngine strict(
        { spec("{ condition: (< gap 0.002), phase: always }") });
    strict.update(robotik::Seconds(1.0), board.probe(strict));
    EXPECT_EQ(strict.checks()[0].verdict, Verdict::Failed);
    EXPECT_FALSE(read("{ condition: (< gap 1), unknown: maybe }"));
}

TEST(Assertions, StopOnFailureIsRequested)
{
    Board board{ { { "ok", 0.0 } } };
    robotik::AssertionEngine engine(
        { spec("{ condition: ok, phase: always, stop: true }") });
    engine.update(robotik::Seconds(0.1), board.probe(engine));
    EXPECT_TRUE(engine.stopRequested());
}

TEST(Assertions, WithinAtAndEventually)
{
    Board board{ { { "x", 0.0 } } };
    robotik::AssertionEngine engine(
        { spec("{ condition: x, within: 2.0 }"),
          spec("{ condition: x, at: 5.0 }"),
          spec("{ condition: x, eventually: true }") });
    engine.update(robotik::Seconds(1.0), board.probe(engine));
    for (robotik::Check const& check : engine.checks())
    {
        EXPECT_EQ(check.verdict, Verdict::Pending);
    }
    engine.update(robotik::Seconds(2.5), board.probe(engine));
    EXPECT_EQ(engine.checks()[0].verdict, Verdict::Failed);
    board.values["x"] = 1.0;
    engine.update(robotik::Seconds(4.0), board.probe(engine));
    EXPECT_EQ(engine.checks()[1].verdict, Verdict::Pending);
    EXPECT_EQ(engine.checks()[2].verdict, Verdict::Passed);
    engine.update(robotik::Seconds(5.0), board.probe(engine));
    EXPECT_EQ(engine.checks()[1].verdict, Verdict::Passed);
}

TEST(Assertions, AtFailsWhenTheRunEndsBefore)
{
    Board board{ { { "x", 1.0 } } };
    robotik::AssertionEngine engine({ spec("{ condition: x, at: 5.0 }") });
    engine.conclude(robotik::Seconds(3.0), board.probe(engine));
    EXPECT_EQ(engine.checks()[0].verdict, Verdict::Failed);
}

TEST(Assertions, SequenceKeepsTheOrder)
{
    Board board{ { { "a", 0.0 }, { "b", 0.0 } } };
    robotik::AssertionEngine good({ spec("{ sequence: [a, b] }") });
    robotik::AssertionEngine bad({ spec("{ sequence: [a, b] }") });
    good.update(robotik::Seconds(0.1), board.probe(good));
    board.values["a"] = 1.0;
    good.update(robotik::Seconds(0.2), board.probe(good));
    board.values["b"] = 1.0;
    good.update(robotik::Seconds(0.3), board.probe(good));
    EXPECT_EQ(good.checks()[0].verdict, Verdict::Passed);

    board.values = { { "a", 0.0 }, { "b", 1.0 } };
    bad.update(robotik::Seconds(0.1), board.probe(bad));
    EXPECT_EQ(bad.checks()[0].verdict, Verdict::Failed);
    EXPECT_NE(bad.checks()[0].detail.find("'b' before 'a'"), std::string::npos);
}

TEST(Assertions, UnknownValueFails)
{
    Board board;
    robotik::AssertionEngine engine({ spec("(> nothing 1)") });
    engine.update(robotik::Seconds(0.1), board.probe(engine));
    EXPECT_EQ(engine.checks()[0].verdict, Verdict::Failed);
    EXPECT_NE(engine.checks()[0].detail.find("nothing"), std::string::npos);
}

TEST(Assertions, InvalidSpecsAreRefused)
{
    EXPECT_FALSE(read("{ phase: always }"));
    EXPECT_FALSE(read("{ condition: x, phase: sometimes }"));
    EXPECT_FALSE(read("{ sequence: [a] }"));
}

TEST(Simulation, AMisplacedCanvasBreaksThePainterAssertions)
{
    // Same world, the canvas hangs 1 m behind the easel, 35 cm above the
    // floor: it falls far from everything. Zero robot collision, yet the
    // scene is wrong.
    robotik::ScenarioSource source = robotik::loadScenarioSource(
        repoFile("plugins/Painter/scenarios/painter.scenario.yaml")).take();
    EXPECT_TRUE(robotik::overrideValue(source, "world.entities.canvas.attach.parent",
                    robotik::ConfigNode("easel")));
    EXPECT_TRUE(robotik::overrideValue(source, "world.entities.canvas.attach.transform.position",
                    robotik::ConfigNode::parse("[-1.0, 0.0, 0.6]").take()));
    PainterLogic logic;
    compages::world::World world;
    robotik::Simulation simulation(
        world,
        robotik::readScenario(source, testAssets()).take(),
        nullptr,
        &logic);
    simulation.cell().suspend(true);
    for (int i = 0; i < 200; ++i)
    {
        simulation.step(robotik::Seconds(0.01));
    }
    EXPECT_EQ(simulation.cell().requireQuery("(on canvas easel)"), 0.0);
    EXPECT_EQ(simulation.cell().requireQuery("(touching canvas easel)"), 0.0);
    EXPECT_GT(*simulation.cell().requireQuery("(pose_error canvas)"), 0.05);
    EXPECT_EQ(simulation.cell().requireQuery("(collision_count robot)"), 0.0);
    EXPECT_THROW((void)simulation.cell().requireQuery("(pose_error nobody)"),
                 std::invalid_argument);
    EXPECT_FALSE(simulation.evaluate("(< (pose_error nobody) 1)").known);
}
