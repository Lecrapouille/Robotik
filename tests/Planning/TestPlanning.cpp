// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "main.hpp"

#include "Robotik/Task/Planning/PlanExecutor.hpp"
#include "Robotik/Task/Planning/PlanningEngine.hpp"
#include "Robotik/Robot/Resources.hpp"
#include "Robotik/Task/Scheduler.hpp"

#include <filesystem>
#include <fstream>

// A gripper moves blocks between places. Typed, with a constant, a
// derived predicate and a quantified effect.
static char const* const BLOCKS = R"(
; Blocks moved by one gripper.
(define (domain Blocks)
  (:requirements :strips :typing :negative-preconditions :derived-predicates
                 :universal-preconditions :conditional-effects)
  (:types block place - object  cube - block)
  (:constants table - place)
  (:predicates (at ?b - block ?p - place) (holding ?b - block) (hand_empty)
               (busy))
  (:derived (busy) (not (hand_empty)))
  (:action pick
    :parameters (?b - block ?p - place)
    :precondition (and (at ?b ?p) (hand_empty))
    :effect (and (holding ?b) (not (at ?b ?p)) (not (hand_empty))))
  (:action place
    :parameters (?b - block ?p - place)
    :precondition (holding ?b)
    :effect (and (at ?b ?p) (hand_empty) (not (holding ?b))))
  (:action drop_all
    :parameters ()
    :effect (forall (?b - block) (not (holding ?b)))))
)";

/// Writes a domain to a temporary file and returns its path.
static std::filesystem::path writeDomain(char const* p_name, char const* p_text)
{
    std::filesystem::path const file =
        std::filesystem::temp_directory_path() / p_name;
    std::ofstream(file) << p_text;
    return file;
}

/// A tiny world: a cube, a bin and a prop the planner ignores.
static robotik::WorldDescription world()
{
    robotik::WorldDescription world;
    world.id = "cell";
    robotik::EntitySpec cube;
    cube.id = "Red_Cube";
    cube.kind = "cube";
    robotik::EntitySpec bin;
    bin.id = "bin";
    bin.kind = "place";
    robotik::EntitySpec decor;
    decor.id = "lamp";
    world.entities = { cube, bin, decor };
    return world;
}

/// Observers for every predicate, bindings for every action.
static robotik::PlanningDomain complete()
{
    robotik::PlanningDomain domain;
    auto unknown = [](robotik::RobotContext const&, robotik::Args)
    { return robotik::Truth::Unknown; };
    domain.observe("at", unknown);
    domain.observe("holding", unknown);
    domain.observe("hand_empty", unknown);
    domain.bind("pick", { "Pick({0})" });
    domain.bind("place", { "Place({1})" }, { { "target", "{1}" } });
    domain.bind("drop_all", {});
    return domain;
}

/// A planner spec whose goal is at(red_cube, bin).
static robotik::PlannerSpec spec(std::filesystem::path const& p_domain)
{
    robotik::PlannerSpec spec;
    spec.domain = p_domain;
    spec.goal = { robotik::Literal::parse("at(red_cube, bin)").take() };
    return spec;
}

/// A blocks problem whose initial state is the given text.
static std::string problemOf(char const* p_init)
{
    return std::string("(define (problem p) (:domain blocks)\n"
                       "  (:objects red_cube - cube bin - place)\n  (:init ") +
           p_init + ")\n  (:goal (at red_cube bin)))";
}

TEST(Planning, ReadsLiterals)
{
    robotik::Literal const in = robotik::Literal::parse(" In(Red_Cube,  box) ").take();
    EXPECT_TRUE(in.positive);
    EXPECT_EQ(in.atom.predicate, "in");
    ASSERT_EQ(in.atom.args.size(), 2u);
    EXPECT_EQ(in.atom.args[0], "red_cube");
    EXPECT_EQ(in.toPddl(), "(in red_cube box)");
    EXPECT_EQ(in.toString(), "in(red_cube, box)");

    robotik::Literal const empty = robotik::Literal::parse("not hand_empty").take();
    EXPECT_FALSE(empty.positive);
    EXPECT_TRUE(empty.atom.args.empty());
    EXPECT_EQ(empty.toPddl(), "(not (hand_empty))");
    EXPECT_FALSE(robotik::Literal::parse("!busy()").take().positive);

    // A flow list [in(a, b)] splits at the comma.
    EXPECT_FALSE(robotik::Literal::parse("in(red_cube"));
    EXPECT_FALSE(robotik::Literal::parse("box)"));
    EXPECT_FALSE(robotik::Literal::parse("in(a b)"));
    EXPECT_FALSE(robotik::Literal::parse("(in a b)"));
}

TEST(Planning, ReadsTheDomainSchema)
{
    robotik::DomainSchema const schema = robotik::DomainSchema::parse(BLOCKS).take();
    EXPECT_EQ(schema.name, "blocks");
    EXPECT_EQ(schema.types.at("cube"), "block");
    EXPECT_TRUE(schema.isA("cube", "block"));
    EXPECT_TRUE(schema.isA("cube", "object"));
    EXPECT_FALSE(schema.isA("cube", "place"));
    EXPECT_EQ(schema.constants.at("table"), "place");

    ASSERT_TRUE(schema.predicates.contains("at"));
    EXPECT_EQ(schema.predicates.at("at").types,
              (std::vector<std::string>{ "block", "place" }));
    EXPECT_TRUE(schema.predicates.at("hand_empty").types.empty());
    EXPECT_TRUE(schema.predicates.at("busy").derived);

    robotik::ActionSchema const& pick = schema.actions.at("pick");
    ASSERT_EQ(pick.parameters.size(), 2u);
    EXPECT_EQ(pick.parameters[0].name, "b");
    EXPECT_EQ(pick.parameters[1].type, "place");
    ASSERT_EQ(pick.effects.size(), 3u);
    EXPECT_EQ(pick.effects[0].toString(), "holding(?b)");
    EXPECT_FALSE(pick.effects[1].positive);
    EXPECT_FALSE(pick.partial);
    EXPECT_TRUE(schema.actions.at("drop_all").partial);

    EXPECT_FALSE(robotik::DomainSchema::parse("(define (domain d)"));
    EXPECT_FALSE(robotik::DomainSchema::parse("(define (problem p))"));
}

TEST(Planning, KeepsTheEndEffectsOfDurativeActions)
{
    robotik::DomainSchema const schema = robotik::DomainSchema::parse(R"(
        (define (domain t) (:requirements :durative-actions)
          (:predicates (moving) (arrived))
          (:durative-action go :parameters () :duration (= ?duration 2)
            :condition (at start (not (arrived)))
            :effect (and (at start (moving)) (at end (not (moving))) (at end (arrived)))))
    )").take();
    robotik::ActionSchema const& go = schema.actions.at("go");
    EXPECT_TRUE(go.durative);
    ASSERT_EQ(go.effects.size(), 2u);
    EXPECT_EQ(go.effects[0].toString(), "not moving");
    EXPECT_EQ(go.effects[1].toString(), "arrived");
}

TEST(Planning, ExpandsBindings)
{
    std::vector<std::string> const args{ "red_cube", "box" };
    EXPECT_EQ(robotik::expand("Transit({1})", args), "Transit(box)");
    EXPECT_EQ(robotik::expand("Move({0},{1})", args), "Move(red_cube,box)");
    EXPECT_EQ(robotik::expand("Home", args), "Home");
    EXPECT_EQ(robotik::expand("{x}", args), "{x}");
    EXPECT_THROW((void)robotik::expand("{2}", args), std::out_of_range);
}

TEST(Planning, KairosSolvesAndReportsErrors)
{
    std::unique_ptr<robotik::PlanningEngine> engine =
        robotik::makeKairosPlanner();
    ASSERT_EQ(engine->load(BLOCKS, "blocks.pddl"), "");

    robotik::PlanningResult const solved =
        engine->solve(problemOf("(at red_cube table) (hand_empty)"), {});
    ASSERT_TRUE(solved.solved()) << solved.diagnostics;
    ASSERT_EQ(solved.steps.size(), 2u);
    EXPECT_EQ(solved.steps[0].toPddl(), "(pick red_cube table)");
    EXPECT_EQ(solved.steps[1].toPddl(), "(place red_cube bin)");
    EXPECT_NE(solved.report.find("(pick red_cube table)"), std::string::npos);

    // Same input, same plan.
    EXPECT_EQ(
        engine->solve(problemOf("(at red_cube table) (hand_empty)"), {}).steps,
        solved.steps);

    // Nothing holds the cube and it is nowhere: no plan.
    robotik::PlanningResult const stuck =
        engine->solve(problemOf("(hand_empty)"), {});
    EXPECT_FALSE(stuck.solved());
    EXPECT_NE(stuck.status, robotik::PlanningResult::Status::Invalid);

    robotik::PlanningResult const bad =
        engine->solve(problemOf("(at red_cube nowhere)"), {});
    EXPECT_EQ(bad.status, robotik::PlanningResult::Status::Invalid);
    EXPECT_NE(bad.diagnostics.find("nowhere"), std::string::npos)
        << bad.diagnostics;

    EXPECT_NE(engine->load("(define (domain d) (:predicates (p))) (:action",
                           "bad.pddl"),
              "");
}

TEST(Planning, ExecutorAcceptsAConsistentSetup)
{
    robotik::ResourceManager resources;
    robotik::SkillScheduler skills(resources);
    bt::NodeFactory factory;
    robotik::PlanningDomain const domain = complete();
    robotik::PlanExecutor const executor(
        spec(writeDomain("robotik_blocks.pddl", BLOCKS)),
        world(),
        domain,
        "My scenario",
        factory,
        skills);
    ASSERT_EQ(executor.objects().size(), 2u);
    EXPECT_EQ(executor.objects()[0].id, "Red_Cube");
    EXPECT_EQ(executor.objects()[0].name, "red_cube");
    EXPECT_EQ(executor.objects()[0].type, "cube");
    EXPECT_EQ(executor.nodes({ "place", { "Red_Cube", "bin" } }),
              (std::vector<std::string>{ "Place(bin)" }));
    EXPECT_EQ(executor.state(), robotik::PlanState::Planning);
}

TEST(Planning, ExecutorListsEveryProblemOfTheSetup)
{
    robotik::ResourceManager resources;
    robotik::SkillScheduler skills(resources);
    bt::NodeFactory factory;
    robotik::PlanningDomain domain = complete();
    domain.observe("hold",
                   [](robotik::RobotContext const&, robotik::Args)
                   { return robotik::Truth::True; });
    domain.bind("place", { "Place({2})" });
    robotik::PlanningDomain partial = domain;
    partial.clear();
    partial.bind("pick", { "Pick({0})" });

    robotik::WorldDescription bad_world = world();
    bad_world.entities[1].kind = "shelf";

    robotik::PlannerSpec bad_spec =
        spec(writeDomain("robotik_blocks.pddl", BLOCKS));
    bad_spec.facts = { robotik::Literal::parse("at(bin, red_cube)").take() };
    bad_spec.goal = { robotik::Literal::parse("in(red_cube, bin)").take(),
                      robotik::Literal::parse("at(red_cube)").take() };

    auto message = [&](robotik::PlannerSpec const& p_spec,
                       robotik::WorldDescription const& p_world,
                       robotik::PlanningDomain const& p_domain)
    {
        try
        {
            robotik::PlanExecutor const executor(
                p_spec, p_world, p_domain, "s", factory, skills);
        }
        catch (std::runtime_error const& error)
        {
            return std::string(error.what());
        }
        return std::string();
    };
    std::string const text = message(bad_spec, bad_world, domain);
    EXPECT_NE(text.find("kind 'shelf'"), std::string::npos) << text;
    EXPECT_NE(text.find("observer 'hold'"), std::string::npos) << text;
    EXPECT_NE(text.find("uses {2}"), std::string::npos) << text;
    EXPECT_NE(text.find("in(red_cube, bin): unknown predicate"),
              std::string::npos)
        << text;
    EXPECT_NE(text.find("expects 2 argument(s)"), std::string::npos) << text;
    EXPECT_NE(text.find("'bin' is a shelf, not a block"), std::string::npos)
        << text;

    std::string const missing = message(
        spec(writeDomain("robotik_blocks.pddl", BLOCKS)), world(), partial);
    EXPECT_NE(missing.find("predicate 'holding' has no observer"),
              std::string::npos)
        << missing;
    EXPECT_NE(missing.find("action 'place' has no binding"), std::string::npos)
        << missing;
    EXPECT_EQ(missing.find("predicate 'busy'"), std::string::npos)
        << "derived: no observer needed";

    std::string const invalid =
        message(spec(writeDomain("robotik_bad.pddl", "(define (domain d)")),
                world(),
                domain);
    EXPECT_NE(invalid.find("robotik_bad.pddl"), std::string::npos) << invalid;
}
