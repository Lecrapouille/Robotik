// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "main.hpp"

#include "RobotIK/Package/StandaloneSimulation.hpp"

#include "RobotIK/Common/Config.hpp"
#include "RobotIK/Package/Simulation.hpp"
#include "RobotIK/Task/Story/Story.hpp"

#include "Compages/World/World.hpp"

#include <cmath>

/// A robot at the origin, a desk, and two people far from it (headless: no
/// model, a box stands in for the second one).
static char const* const HALL = R"(
world:
  id: hall
  entities:
    - { id: robot, type: robot, model: robotik.robots.robot_6axis }
    - { id: desk, type: prop, shape: { type: box, size: [0.8, 0.4, 0.75] },
        pose: { position: [1.5, 0.0, 0.375] } }
    - id: visitor
      type: actor
      pose: { position: [6.0, 0.0, 0.0], rpy: [0.0, 0.0, 3.14159] }
      actor: { height: 1.8, walk_speed: 1.2, run_speed: 3.0 }
    - id: child
      type: actor
      shape: { type: box, size: [0.3, 0.3, 1.2] }
      pose: { position: [6.0, 2.0, 0.0] }
      actor: { height: 1.2, walk_speed: 0.9 }
)";

/// A scenario with this story and these assertions, read as a file would be.
static robotik::Result<robotik::Scenario> scenarioWith(std::string const& p_body)
{
    robotik::ScenarioSource source;
    source.scenario = robotik::ConfigNode::parse("scenario:\n  id: story\n" + p_body).take();
    source.world = robotik::ConfigNode::parse(HALL).take();
    source.package = ".";
    return robotik::readScenario(source, testAssets());
}

/// Steps until the run ends or @p_limit.
static void runFor(robotik::Simulation& p_simulation, double p_limit)
{
    while (!p_simulation.finished() && p_simulation.time() < robotik::Seconds(p_limit))
    {
        p_simulation.step(robotik::Seconds(0.01));
    }
    p_simulation.conclude();
}

/// The log of the story, one line each, for a failed expectation.
static std::string logOf(robotik::Simulation const& p_simulation)
{
    std::string text;
    for (std::string const& line : p_simulation.cell().story()->log())
    {
        text += line + '\n';
    }
    return text;
}

TEST(Story, ReadsEveryActionKeyword)
{
    robotik::Scenario const scenario = scenarioWith(R"(
  story:
    places: { door: [6.0, 0.0], desk_front: [2.2, 0.0] }
    variables: { checked_in: 0, queue: 0 }
    events:
      - id: everything
        actor: visitor
        start: (> time 1)
        after: other
        delay: 0.5
        repeat: forever
        interrupt: true
        stop_when: (> time 50)
        actions:
          - walk_to: desk_front
          - run_to: { target: robot, distance: 1.2, timeout: 9 }
          - follow_path: { points: [door, [3.0, 1.0], robot.tool0], loop: true, gait: run, speed: 2.0 }
          - follow: { target: child, distance: 1.5, until: (> time 9) }
          - turn_to: robot
          - turn_to: { heading: 1.57 }
          - wait: [1, 3]
          - wait_until: { condition: (event_done other), timeout: 4 }
          - say: { id: hello, text: "Hello, I have a meeting", to: robot }
          - say: Thank you
          - play: { clip: wave, duration: 1.5, loop: true }
          - expression: { name: smile, weight: 0.7, duration: 2 }
          - teleport: { position: [6.0, 0.0, 0.0], rpy: [0, 0, 3.14] }
          - show: false
          - hide: false
          - set: { checked_in: 1 }
          - add: { queue: -1 }
          - start: [other]
          - stop: other
          - choose:
              - { weight: 2, actions: [ { wait: 1 } ] }
              - { actions: [ { say: Bye } ] }
      - id: other
        actions: [ { wait: 1 } ]
)").take();
    robotik::StorySpec const& story = scenario.story;
    ASSERT_EQ(story.events.size(), 2U);
    EXPECT_DOUBLE_EQ(story.places.at("desk_front").x, 2.2);
    EXPECT_DOUBLE_EQ(story.variables.at("checked_in"), 0.0);
    robotik::StoryEvent const& event = story.events[0];
    EXPECT_EQ(event.repeat, 0U);
    EXPECT_TRUE(event.interrupt);
    EXPECT_EQ(event.after, std::vector<std::string>{ "other" });
    using Type = robotik::StoryAction::Type;
    std::vector<Type> const types = { Type::WalkTo,   Type::WalkTo,     Type::FollowPath,
                                      Type::Follow,   Type::TurnTo,     Type::TurnTo,
                                      Type::Wait,     Type::WaitUntil,  Type::Say,
                                      Type::Say,      Type::Play,       Type::Expression,
                                      Type::Teleport, Type::Show,       Type::Show,
                                      Type::Set,      Type::Add,        Type::Start,
                                      Type::Stop,     Type::Choose };
    ASSERT_EQ(event.actions.size(), types.size());
    for (std::size_t i = 0; i < types.size(); ++i)
    {
        EXPECT_EQ(event.actions[i].type, types[i]) << event.actions[i].where;
    }
    EXPECT_EQ(event.actions[1].gait, robotik::Gait::Run);
    EXPECT_DOUBLE_EQ(event.actions[1].distance, 1.2);
    ASSERT_EQ(event.actions[2].targets.size(), 3U);
    EXPECT_TRUE(event.actions[2].targets[1].point.has_value());
    EXPECT_DOUBLE_EQ(*event.actions[2].speed, 2.0);
    EXPECT_DOUBLE_EQ(*event.actions[5].heading, 1.57);
    EXPECT_DOUBLE_EQ(event.actions[6].duration_max, 3.0);
    EXPECT_EQ(event.actions[8].name, "hello");
    EXPECT_EQ(event.actions[9].name, "everything_10") << "a say without id gets one";
    EXPECT_DOUBLE_EQ(event.actions[13].value, 0.0) << "show: false";
    EXPECT_DOUBLE_EQ(event.actions[14].value, 1.0) << "hide: false shows";
    ASSERT_EQ(event.actions[19].branches.size(), 2U);
    EXPECT_DOUBLE_EQ(event.actions[19].branches[0].weight, 2.0);
}

TEST(Story, LoadErrorsNameTheEventAndTheAction)
{
    auto error = [](std::string const& p_story)
    {
        robotik::Result<robotik::Scenario> const read = scenarioWith(p_story);
        return read ? std::string() : read.error();
    };
    std::string const unknown = error(R"(
  story:
    events: [ { id: a, actor: visitor, actions: [ { jump: robot } ] } ]
)");
    EXPECT_NE(unknown.find("event 'a', action 1 (jump)"), std::string::npos) << unknown;
    EXPECT_NE(unknown.find("walk_to"), std::string::npos) << "lists the known actions: " << unknown;

    EXPECT_NE(error(R"(
  story:
    events: [ { id: a, actor: desk, actions: [ { wait: 1 } ] } ]
)").find("is no entity of type actor"), std::string::npos);
    EXPECT_NE(error(R"(
  story:
    events: [ { id: a, actor: visitor, actions: [ { walk_to: nowhere } ] } ]
)").find("'nowhere' is no place nor entity"), std::string::npos);
    EXPECT_NE(error(R"(
  story:
    events: [ { id: a, actions: [ { set: { score: 1 } } ] } ]
)").find("not declared in story.variables"), std::string::npos);
    EXPECT_NE(error(R"(
  story:
    events: [ { id: a, actions: [ { say: hi } ] } ]
)").find("needs the 'actor:'"), std::string::npos);
    EXPECT_NE(error(R"(
  story:
    events: [ { id: a, strat: (> time 1), actions: [ { wait: 1 } ] } ]
)").find("did you mean 'start'"), std::string::npos);

    // A condition is compiled when the cell is built: a typo is a load error.
    robotik::Scenario const typo = scenarioWith(R"(
  story:
    events: [ { id: a, start: (> tme 1), actions: [ { wait: 1 } ] } ]
)").take();
    compages::world::World world;
    EXPECT_THROW(robotik::Simulation(world, typo), std::invalid_argument);
}

TEST(Story, AnActorWalksToAPlaceTurnsAndSpeaks)
{
    robotik::Scenario const scenario = scenarioWith(R"(
  timeout: 20
  stop_when: (event_done arrive)
  story:
    places: { desk_front: [2.2, 0.0] }
    events:
      - id: arrive
        actor: visitor
        start: (> time 1)
        actions:
          - walk_to: desk_front
          - turn_to: robot
          - say: { id: hello, text: "Hello, I have a meeting at ten", to: robot }
  assertions:
    - { condition: (event_done arrive), eventually: true }
    - { condition: (said visitor hello), eventually: true }
    - { condition: (at visitor desk_front 0.15) }
    - { condition: (< (heading_error visitor robot) 0.1) }
    - { condition: (not (event_started arrive)), at: 0.9 }
    - { condition: (> (actor_speed visitor) 1.0), at: 3.0 }
)").take();
    auto opened =
        robotik::require(robotik::StandaloneSimulation::open(scenario));
    robotik::Simulation& simulation = opened.simulation();
    simulation.reset(robotik::Seed{ 1U });
    runFor(simulation, 20.0);
    EXPECT_TRUE(simulation.passed()) << logOf(simulation);
    // 3.8 m at 1.2 m/s, after 1 s: then the turn and six words at 2.5 a second.
    EXPECT_GT(simulation.time().value(), 1.0 + 3.8 / 1.2);
    EXPECT_LT(simulation.time().value(), 10.0);
    ASSERT_EQ(simulation.cell().story()->utterances().size(), 1U);
    EXPECT_EQ(simulation.cell().story()->utterances()[0].to, "robot");
}

TEST(Story, EventsChainRepeatAndCancel)
{
    robotik::Scenario const scenario = scenarioWith(R"(
  timeout: 30
  story:
    variables: { visits: 0 }
    events:
      # No start: at once, three times in a row; each run waits 1 s and counts.
      - id: tick
        repeat: 3
        actions: [ { wait: 1 }, { add: { visits: 1 } } ]
      # Rising edge: starts again only when its condition falls and rises.
      - id: edge
        start: (> (story_var visits) 0)
        repeat: forever
        actions: [ { wait: 0.1 } ]
      - id: after_two
        start: (>= (story_var visits) 2)
        after: tick
        actions: [ { start: walk }, { wait: 0.5 } ]
      - id: walk
        actor: visitor
        start: (> time 100)
        actions: [ { walk_to: [0.0, 5.0] } ]
      - id: stop_walk
        start: (> time 4)
        actions: [ { stop: walk } ]
  assertions:
    - { condition: (= (event_count tick) 1), at: 1.5 }
    - { condition: (= (story_var visits) 1), at: 1.5 }
    - { condition: (= (event_count edge) 1), at: 5.0 }
    - { condition: (event_running walk), at: 3.0 }
    - { condition: (event_failed walk), at: 4.5 }
    - { condition: (= (actor_speed visitor) 0), at: 5.0 }
    - { condition: (= (event_count tick) 3), at: 5.0 }
)").take();
    auto opened =
        robotik::require(robotik::StandaloneSimulation::open(scenario));
    robotik::Simulation& simulation = opened.simulation();
    simulation.reset(robotik::Seed{ 1U });
    runFor(simulation, 6.0);
    EXPECT_TRUE(simulation.passed()) << logOf(simulation);
}

TEST(Story, AnActorPlaysOneEventAtATime)
{
    robotik::Scenario const scenario = scenarioWith(R"(
  story:
    events:
      - id: long
        actor: child
        actions: [ { wait: 3 } ]
      - id: polite
        actor: child
        start: (> time 1)
        actions: [ { wait: 0.5 } ]
      - id: rude
        actor: visitor
        actions: [ { wait: 3 } ]
      - id: urgent
        actor: visitor
        start: (> time 1)
        interrupt: true
        actions: [ { wait: 0.5 } ]
  assertions:
    # polite waits for long; urgent cancels rude.
    - { condition: (and (event_running long) (not (event_running polite))), at: 2.0 }
    - { condition: (event_done polite), at: 3.6 }
    - { condition: (and (event_failed rude) (event_done urgent)), at: 2.0 }
)").take();
    auto opened =
        robotik::require(robotik::StandaloneSimulation::open(scenario));
    robotik::Simulation& simulation = opened.simulation();
    simulation.reset(robotik::Seed{ 1U });
    runFor(simulation, 4.0);
    EXPECT_TRUE(simulation.passed()) << logOf(simulation);
}

TEST(Story, StopWhenAndTimeoutEndAnEvent)
{
    robotik::Scenario const scenario = scenarioWith(R"(
  story:
    events:
      - id: patient
        actions: [ { wait_until: { condition: (> time 100), timeout: 1 } } ]
      - id: wanderer
        actor: visitor
        stop_when: (> time 0.5)
        actions: [ { follow_path: { points: [[6, 3], [3, 3]], loop: true } } ]
      - id: follower
        actor: child
        actions: [ { follow: { target: visitor, distance: 1.0, until: (> time 3) } } ]
  assertions:
    - { condition: (event_failed patient), at: 1.5 }
    - { condition: (and (event_failed wanderer) (= (actor_speed visitor) 0)), at: 1.0 }
    - { condition: (< (distance child visitor) 1.3), at: 2.9 }
    - { condition: (event_done follower), at: 3.2 }
)").take();
    auto opened =
        robotik::require(robotik::StandaloneSimulation::open(scenario));
    robotik::Simulation& simulation = opened.simulation();
    simulation.reset(robotik::Seed{ 1U });
    runFor(simulation, 3.5);
    EXPECT_TRUE(simulation.passed()) << logOf(simulation);
}

TEST(Story, InstantActionsChangeTheWorldAtOnce)
{
    robotik::Scenario const scenario = scenarioWith(R"(
  story:
    variables: { checked_in: 0 }
    events:
      - id: instant
        actor: visitor
        actions:
          - teleport: { position: [1.0, 2.0, 0.0], rpy: [0, 0, 1.5708] }
          - expression: { name: smile, weight: 0.8 }
          - set: { checked_in: 1 }
          - hide: true
          - play: { clip: wave, duration: 1 }
)").take();
    auto opened =
        robotik::require(robotik::StandaloneSimulation::open(scenario));
    robotik::Simulation& simulation = opened.simulation();
    simulation.reset(robotik::Seed{ 1U });
    simulation.step(robotik::Seconds(0.01));
    robotik::RoboticCell const& cell = simulation.cell();
    EXPECT_NEAR(cell.requirePose("visitor").position.x, 1.0, 1e-5);
    EXPECT_NEAR(cell.requirePose("visitor").position.y, 2.0, 1e-5);
    EXPECT_NEAR(cell.requirePose("visitor.head").position.z, 0.93 * 1.8, 1e-5);
    EXPECT_DOUBLE_EQ(cell.requireQuery("(expression visitor smile)").value(), 0.8);
    EXPECT_DOUBLE_EQ(cell.requireQuery("(story_var checked_in)").value(), 1.0);
    EXPECT_DOUBLE_EQ(cell.requireQuery("(shown visitor)").value(), 0.0);
    EXPECT_DOUBLE_EQ(cell.requireQuery("(event_running instant)").value(), 1.0)
        << "play lasts 1 s";
}

TEST(Story, TheSameSeedGivesTheSameStory)
{
    robotik::Scenario const scenario = scenarioWith(R"(
  story:
    variables: { left: 0 }
    events:
      - id: drawn
        actor: visitor
        actions:
          - wait: [0.5, 2.0]
          - choose:
              - { actions: [ { set: { left: 1 } }, { walk_to: [6.0, 4.0] } ] }
              - { actions: [ { set: { left: 2 } }, { walk_to: [6.0, -4.0] } ] }
)").take();
    auto runWith = [&](std::uint64_t p_seed)
    {
        auto opened =
            robotik::require(robotik::StandaloneSimulation::open(scenario));
        robotik::Simulation& simulation = opened.simulation();
        simulation.reset(robotik::Seed{ p_seed });
        runFor(simulation, 6.0);
        std::string trace = logOf(simulation);
        trace += std::to_string(simulation.cell().requirePose("visitor").position.y);
        return trace;
    };
    EXPECT_EQ(runWith(4U), runWith(4U));
    bool differs = false;
    for (std::uint64_t seed = 5U; seed < 12U && !differs; ++seed)
    {
        differs = runWith(seed) != runWith(4U);
    }
    EXPECT_TRUE(differs) << "another seed draws another time or branch";
}
