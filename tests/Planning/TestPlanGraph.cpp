// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "main.hpp"

#include "Robotik/Planning/PlanGraph.hpp"

/// Arms that pick and place; one hand each.
static char const* const ARMS = R"(
(define (domain arms)
  (:requirements :typing :negative-preconditions)
  (:types arm part bin)
  (:predicates (free ?r - arm) (holding ?r - arm ?p - part)
               (at ?p - part ?b - bin) (loose ?p - part))
  (:action pick :parameters (?r - arm ?p - part)
    :precondition (and (free ?r) (loose ?p))
    :effect (and (holding ?r ?p) (not (free ?r)) (not (loose ?p))))
  (:action place :parameters (?r - arm ?p - part ?b - bin)
    :precondition (holding ?r ?p)
    :effect (and (at ?p ?b) (free ?r) (not (holding ?r ?p))))
  (:durative-action wait :parameters (?r - arm)
    :duration (= ?duration 1)
    :condition (over all (free ?r))
    :effect (at end (free ?r))))
)";

/// Footprints of @p_steps in the domain @ref ARMS.
static std::vector<robotik::StepFootprint> prints(std::vector<robotik::GroundAction> const& p_steps)
{
    robotik::DomainSchema const schema = robotik::DomainSchema::parse(ARMS);
    std::vector<robotik::StepFootprint> result;
    for (robotik::GroundAction const& step : p_steps)
    {
        result.push_back(robotik::footprint(schema.actions.at(step.name), step));
    }
    return result;
}

TEST(PlanGraph, SchemaReadsPreconditionsAndWrites)
{
    robotik::DomainSchema const schema = robotik::DomainSchema::parse(ARMS);
    robotik::ActionSchema const& pick = schema.actions.at("pick");
    ASSERT_EQ(pick.reads.size(), 2u);
    EXPECT_EQ(pick.reads[0].predicate, "free");
    EXPECT_EQ(pick.writes.size(), 3u);
    // (at ?p ?b) is an atom, (over all ...) and (at end ...) are not.
    EXPECT_EQ(schema.actions.at("place").writes[0].predicate, "at");
    ASSERT_EQ(schema.actions.at("wait").reads.size(), 1u);
    EXPECT_EQ(schema.actions.at("wait").reads[0].predicate, "free");
    EXPECT_EQ(schema.actions.at("wait").writes[0].predicate, "free");
}

TEST(PlanGraph, OneArmStaysInOrder)
{
    auto const after = robotik::orderSteps(prints({ { "pick", { "left", "a" } },
                                                    { "place", { "left", "a", "bin" } },
                                                    { "pick", { "left", "b" } },
                                                    { "place", { "left", "b", "bin" } } }));
    EXPECT_TRUE(robotik::sequential(after));
}

TEST(PlanGraph, TwoArmsWorkTogether)
{
    // left: 0 -> 2, right: 1 -> 3; the two chains are independent.
    auto const after = robotik::orderSteps(prints({ { "pick", { "left", "a" } },
                                                    { "pick", { "right", "b" } },
                                                    { "place", { "left", "a", "bin" } },
                                                    { "place", { "right", "b", "bin" } } }));
    ASSERT_EQ(after.size(), 4u);
    EXPECT_TRUE(after[0].empty());
    EXPECT_TRUE(after[1].empty());
    EXPECT_EQ(after[2], std::vector<std::size_t>{ 0u });
    EXPECT_EQ(after[3], std::vector<std::size_t>{ 1u });
    EXPECT_FALSE(robotik::sequential(after));
}

TEST(PlanGraph, SharedResourceKeepsTheOrder)
{
    std::vector<robotik::StepFootprint> steps = prints({ { "pick", { "left", "a" } },
                                                         { "pick", { "right", "b" } } });
    // Both skills need the camera alone: no overlap.
    steps[0].resources = { { 3u, robotik::Access::Exclusive } };
    steps[1].resources = { { 3u, robotik::Access::Shared } };
    EXPECT_EQ(robotik::orderSteps(steps)[1], std::vector<std::size_t>{ 0u });
    // Both only watch the stream: they may share it.
    steps[0].resources = { { 3u, robotik::Access::Shared } };
    EXPECT_TRUE(robotik::orderSteps(steps)[1].empty());
}

TEST(PlanGraph, ImpliedPairsAreLeftOut)
{
    // 0 -> 1 -> 2 with left; 0 -> 2 is implied, not listed.
    auto const after = robotik::orderSteps(prints({ { "pick", { "left", "a" } },
                                                    { "place", { "left", "a", "bin" } },
                                                    { "pick", { "left", "b" } } }));
    EXPECT_EQ(after[2], std::vector<std::size_t>{ 1u });
}
