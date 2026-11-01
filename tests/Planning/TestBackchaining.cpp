// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

// The tree of PA-BT grown backwards from the goal (Backchaining.hpp).

#include "main.hpp"

#include "Robotik/Task/Planning/Backchaining.hpp"

#include <fstream>
#include <sstream>

/// Text of a file of the repository.
static std::string text(char const* p_file)
{
    std::ifstream file(repoFile(p_file));
    std::ostringstream content;
    content << file.rdbuf();
    return content.str();
}

/// The objects of the conveyor cell, by PDDL name.
static std::map<std::string, std::string, std::less<>> conveyorObjects()
{
    return { { "left", "arm" },        { "right", "arm" },
             { "feeder", "feeder" },   { "zone_left", "zone" },
             { "zone_right", "zone" }, { "p1", "part" },
             { "bin_left", "bin" },    { "bin_right", "bin" } };
}

/// The actions of @p_actions, as PDDL text.
static std::vector<std::string>
pddl(std::vector<robotik::GroundAction> const& p_actions)
{
    std::vector<std::string> result;
    for (robotik::GroundAction const& action : p_actions)
    {
        result.push_back(robotik::toPddl(action));
    }
    return result;
}

TEST(Backchaining, ReadsThePlainPreconditions)
{
    robotik::DomainSchema const schema = robotik::parseDomain(R"(
        (define (domain t) (:requirements :adl :durative-actions)
          (:predicates (clean) (loaded ?c) (painted ?c) (before ?a ?b))
          (:action mark :parameters (?c)
            :precondition (and (loaded ?c) (not (painted ?c))
                               (forall (?p) (imply (before ?p ?c) (painted ?p))))
            :effect (painted ?c))
          (:durative-action wash :parameters () :duration (= ?duration 2)
            :condition (and (at start (not (clean))) (over all (loaded ?c))
                            (at end (clean)))
            :effect (at end (clean))))
    )")
                                             .take();
    robotik::ActionSchema const& mark = schema.actions.at("mark");
    ASSERT_EQ(mark.preconditions.size(), 2U);
    EXPECT_EQ(robotik::toString(mark.preconditions[0]), "loaded(?c)");
    EXPECT_EQ(robotik::toString(mark.preconditions[1]), "not painted(?c)");
    EXPECT_TRUE(mark.partial_preconditions); // the forall

    robotik::ActionSchema const& wash = schema.actions.at("wash");
    ASSERT_EQ(wash.preconditions.size(), 2U);
    EXPECT_EQ(robotik::toString(wash.preconditions[0]), "not clean");
    EXPECT_EQ(robotik::toString(wash.preconditions[1]), "loaded(?c)");
    EXPECT_TRUE(wash.partial_preconditions); // the at end
}

TEST(Backchaining, FindsTheActionsThatMakeALiteralTrue)
{
    robotik::DomainSchema const schema =
        robotik::parseDomain(
            text("plugins/DualArm/scenarios/conveyor.domain.pddl"))
            .take();
    auto const objects = conveyorObjects();

    // (at ?p ?b) is the predicate at, not a timed effect.
    robotik::ActionSchema const& place = schema.actions.at("place");
    ASSERT_EQ(place.effects.size(), 3U);
    EXPECT_EQ(robotik::toString(place.effects[0]), "at(?p, ?b)");
    EXPECT_EQ(robotik::toString(schema.actions.at("convey").preconditions[0]),
              "at(?p, ?f)");

    // convey makes at(p1, z) for a zone only. place(?r ?p ?b ?z) puts the part in the bin: the arm and the zone
    // take every object of their type.
    robotik::Literal const in_bin =
        robotik::parseLiteral("at(p1, bin_left)").take();
    EXPECT_EQ(
        pddl(robotik::achievers(schema, objects, in_bin)),
        (std::vector<std::string>{ "(place left p1 bin_left zone_left)",
                                   "(place left p1 bin_left zone_right)",
                                   "(place right p1 bin_left zone_left)",
                                   "(place right p1 bin_left zone_right)" }));

    // A negative literal: what the delete effects make true.
    robotik::Literal const not_free =
        robotik::parseLiteral("not free(left)").take();
    EXPECT_EQ(pddl(robotik::achievers(schema, objects, not_free, 2U)),
              (std::vector<std::string>{ "(pick left p1 zone_left)",
                                         "(pick left p1 zone_right)" }));

    EXPECT_EQ(robotik::staticPredicates(schema),
              (std::set<std::string, std::less<>>{ "near", "reaches" }));
    EXPECT_EQ(robotik::toString(robotik::groundPreconditions(
                  schema,
                  { "place",
                    { "left", "p1", "bin_left", "zone_left" },
                    0.0,
                    0.0 })[2]),
              "reaches(left, zone_left)");
}

TEST(Backchaining, GrowsTheTreeAndKnowsTheAncestors)
{
    robotik::BackchainTree tree =
        robotik::backchainTree({ robotik::parseLiteral("in(cube, box)").take(),
                                 robotik::parseLiteral("hand_empty").take() });
    ASSERT_EQ(tree.goal, (std::vector<std::size_t>{ 0U, 1U }));
    std::size_t const place =
        robotik::attach(tree,
                        0U,
                        { "place", { "cube", "box" }, 0.0, 0.0 },
                        { robotik::parseLiteral("holding(cube)").take() });
    ASSERT_EQ(tree.conditions.size(), 3U);
    EXPECT_EQ(tree.conditions[0].achievers,
              (std::vector<std::size_t>{ place }));
    EXPECT_EQ(tree.conditions[2].action, place);
    std::size_t const pick =
        robotik::attach(tree,
                        2U,
                        { "pick", { "cube" }, 0.0, 0.0 },
                        { robotik::parseLiteral("hand_empty").take() });
    std::vector<robotik::Literal> const above =
        robotik::ancestors(tree, tree.actions[pick].preconditions[0]);
    ASSERT_EQ(above.size(), 3U);
    EXPECT_EQ(robotik::toString(above[0]), "hand_empty");
    EXPECT_EQ(robotik::toString(above[1]), "holding(cube)");
    EXPECT_EQ(robotik::toString(above[2]), "in(cube, box)");
    EXPECT_EQ(robotik::ancestors(tree, 1U).size(), 1U); // a goal
}
