// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "main.hpp"

#include "Robotik/Task/Planning/PlanGraph.hpp"
#include "Robotik/Task/Planning/PlanningEngine.hpp"

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
    :effect (and (at ?p ?b) (free ?r) (not (holding ?r ?p)))))
)";

/// One arm carries a tray while the other one visits the glasses on it:
/// visit needs the tray held all along (required concurrency).
static char const* const TRAY = R"(
(define (domain tray)
  (:requirements :typing :durative-actions)
  (:types arm tray glass)
  (:predicates (held ?t - tray) (carried ?t - tray) (visited ?g - glass)
               (on ?g - glass ?t - tray))
  (:durative-action carry :parameters (?r - arm ?t - tray)
    :duration (= ?duration 10)
    :condition (at start (on g1 ?t))
    :effect (and (at start (held ?t)) (at end (not (held ?t))) (at end (carried ?t))))
  (:durative-action visit :parameters (?r - arm ?g - glass ?t - tray)
    :duration (= ?duration 2)
    :condition (and (over all (held ?t)) (over all (on ?g ?t)))
    :effect (at end (visited ?g))))
)";

/// Footprints of @p_steps in @p_domain.
static std::vector<robotik::StepFootprint>
prints(char const* p_domain, std::vector<robotik::GroundAction> const& p_steps)
{
    robotik::DomainSchema const schema = robotik::parseDomain(p_domain).take();
    std::vector<robotik::StepFootprint> result;
    for (robotik::GroundAction const& step : p_steps)
    {
        result.push_back(
            robotik::footprint(schema.actions.at(step.name), step));
    }
    return result;
}

/// A step of a sequential plan, at position @p_at.
static robotik::GroundAction
at(std::size_t p_at, std::string p_name, std::vector<std::string> p_args)
{
    return {
        std::move(p_name), std::move(p_args), static_cast<double>(p_at), 0.0
    };
}

/// End of step @p_step.
static robotik::PlanEvent end(std::size_t p_step)
{
    return { p_step, true };
}

TEST(PlanGraph, SchemaReadsEachEnd)
{
    robotik::DomainSchema const arms = robotik::parseDomain(ARMS).take();
    robotik::ActionSchema const& pick = arms.actions.at("pick");
    // A plain action reads at its start and changes at its end.
    ASSERT_EQ(pick.start.reads.size(), 2U);
    EXPECT_EQ(pick.start.reads[0].predicate, "free");
    EXPECT_TRUE(pick.start.writes.empty());
    EXPECT_EQ(pick.end.writes.size(), 3U);
    // (at ?p ?b) is an atom, not a time.
    EXPECT_EQ(arms.actions.at("place").end.writes[0].predicate, "at");

    robotik::DomainSchema const tray = robotik::parseDomain(TRAY).take();
    robotik::ActionSchema const& carry = tray.actions.at("carry");
    ASSERT_EQ(carry.start.writes.size(), 1U);
    EXPECT_EQ(carry.start.writes[0].predicate, "held");
    EXPECT_EQ(carry.end.writes.size(), 2U);
    // over all: read at both ends.
    robotik::ActionSchema const& visit = tray.actions.at("visit");
    EXPECT_EQ(visit.start.reads.size(), 2U);
    EXPECT_EQ(visit.end.reads.size(), 2U);
}

TEST(PlanGraph, OneArmStaysInOrder)
{
    auto const order =
        robotik::orderSteps(prints(ARMS,
                                   { at(0, "pick", { "left", "a" }),
                                     at(1, "place", { "left", "a", "bin" }),
                                     at(2, "pick", { "left", "b" }),
                                     at(3, "place", { "left", "b", "bin" }) }));
    EXPECT_TRUE(robotik::sequential(order));
}

TEST(PlanGraph, TwoArmsWorkTogether)
{
    // left: 0 -> 2, right: 1 -> 3; the two chains are independent.
    auto const order = robotik::orderSteps(
        prints(ARMS,
               { at(0, "pick", { "left", "a" }),
                 at(1, "pick", { "right", "b" }),
                 at(2, "place", { "left", "a", "bin" }),
                 at(3, "place", { "right", "b", "bin" }) }));
    ASSERT_EQ(order.size(), 4U);
    EXPECT_TRUE(order[0].start_after.empty());
    EXPECT_TRUE(order[1].start_after.empty());
    EXPECT_EQ(order[2].start_after, std::vector<robotik::PlanEvent>{ end(0) });
    EXPECT_EQ(order[3].start_after, std::vector<robotik::PlanEvent>{ end(1) });
    EXPECT_FALSE(robotik::sequential(order));
}

TEST(PlanGraph, SharedResourceKeepsTheOrder)
{
    std::vector<robotik::StepFootprint> steps = prints(
        ARMS,
        { at(0, "pick", { "left", "a" }), at(1, "pick", { "right", "b" }) });
    // One needs the camera alone: no overlap.
    steps[0].resources = { { 3U, robotik::Access::Exclusive } };
    steps[1].resources = { { 3U, robotik::Access::Shared } };
    EXPECT_EQ(robotik::orderSteps(steps)[1].start_after,
              std::vector<robotik::PlanEvent>{ end(0) });
    // Both only watch the stream: they may share it.
    steps[0].resources = { { 3U, robotik::Access::Shared } };
    EXPECT_TRUE(robotik::orderSteps(steps)[1].start_after.empty());
}

TEST(PlanGraph, ImpliedPairsAreLeftOut)
{
    // 0 -> 1 -> 2 with left; 0 -> 2 is implied, not listed.
    auto const order =
        robotik::orderSteps(prints(ARMS,
                                   { at(0, "pick", { "left", "a" }),
                                     at(1, "place", { "left", "a", "bin" }),
                                     at(2, "pick", { "left", "b" }) }));
    EXPECT_EQ(order[2].start_after, std::vector<robotik::PlanEvent>{ end(1) });
}

TEST(PlanGraph, HeldAllAlongTheVisit)
{
    // carry [0, 10], visit [1, 3]: visit starts once carry started, and
    // carry ends once visit ended.
    auto const order = robotik::orderSteps(
        prints(TRAY,
               { { "carry", { "left", "tray" }, 0.0, 10.0 },
                 { "visit", { "right", "g1", "tray" }, 1.0, 2.0 } }));
    EXPECT_EQ(order[1].start_after,
              (std::vector<robotik::PlanEvent>{ { 0U, false } }));
    EXPECT_EQ(order[0].end_after, (std::vector<robotik::PlanEvent>{ end(1) }));
    EXPECT_FALSE(robotik::sequential(order));
}

TEST(PlanGraph, KairosGivesTheTimesOfATemporalPlan)
{
    std::unique_ptr<robotik::PlanningEngine> engine =
        robotik::makeKairosPlanner();
    ASSERT_EQ(engine->load(TRAY, "tray.pddl"), "");
    robotik::PlanningResult const result = engine->solve(R"(
(define (problem serve) (:domain tray)
  (:objects left right - arm tray - tray g1 g2 - glass)
  (:init (on g1 tray) (on g2 tray))
  (:goal (and (visited g1) (visited g2) (carried tray))))
)",
                                                         {});
    ASSERT_TRUE(solved(result)) << result.diagnostics << result.report;
    ASSERT_EQ(result.steps.size(), 3U);
    robotik::GroundAction const& carry = result.steps.front();
    EXPECT_EQ(carry.name, "carry");
    EXPECT_DOUBLE_EQ(carry.duration, 10.0);
    for (std::size_t k = 1U; k < 3U; ++k)
    {
        // Each visit lies within the carry.
        EXPECT_EQ(result.steps[k].name, "visit");
        EXPECT_GE(result.steps[k].start, carry.start);
        EXPECT_LT(result.steps[k].start + result.steps[k].duration,
                  carry.start + carry.duration);
    }
}
