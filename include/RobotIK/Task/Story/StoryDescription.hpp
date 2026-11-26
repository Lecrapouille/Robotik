// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file StoryDescription.hpp
/// What the actors of a world do during a scenario, written as data: the
/// people of a hall, a forklift, a dog. They are not robots: no skill drives
/// them, no physics moves them. A *story* is a list of *events*; an event
/// starts when its condition becomes true and plays a list of *actions*
/// (walk to a place, turn to the robot, say a sentence, wait...). The idea
/// comes from the storyboard of ASAM OpenSCENARIO, used to test cars.
///
/// The Package layer reads it from the @c story: section of a scenario file
/// (doc/reference/story-file.md); a program may also fill it in C++.
///
/// @par Contract
/// - Promises: plain data; nothing here reads a file or moves an actor.
/// - Expects: ids of events are unique; an action that moves names an actor
///   of type @c actor in the world (checked when the cell is built).
/// - Errors: none here; the reader and @ref robotik::Story return a message.
#pragma once

#include "RobotIK/Common/Math.hpp"

#include <array>
#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace robotik
{

// ****************************************************************************
/// How an actor of the world looks and moves (@c actor: of an entity of type
/// @c actor).
///
/// Why: a person walks on arcs at a few speeds, has a head the robot looks
/// at, and plays clips (Idle, Walk, Run, Wave) when a view draws it. These
/// numbers are the same for every story it plays in.
///
/// @code{.yaml}
/// - id: visitor
///   type: actor
///   model: robotik.people.soldier      # a glTF; optional (headless: none)
///   pose: { position: [6, 0, 0], rpy: [0, 0, 3.14] }
///   actor:
///     height: 1.8
///     walk_speed: 1.3
///     run_speed: 3.0
///     model_yaw: 1.5708                # rad, so that it faces +X
///     clips: { idle: Idle, walk: Walk, run: Run, talk: Idle }
///     frames: { head: [0, 0, 1.65] }
/// @endcode
///
/// The actor faces +X of its own frame and turns about Z, like a mobile
/// robot. Limits: no physics, so a robot passes through it and a lidar does
/// not see it yet (ROADMAP-HRI.md).
// ****************************************************************************
struct ActorSpec
{
    /// Height of the body (m); places the default @c head frame.
    double height = 1.75;
    /// Radius of the body seen from above (m): @c (at a b) counts from it.
    double radius = 0.25;
    /// Speed of @c walk (m/s).
    double walk_speed = 1.3;
    /// Speed of @c run (m/s).
    double run_speed = 3.0;
    /// Fastest turn (rad/s).
    double turn_rate = 3.0;
    /// Tightest circle it walks on (m). The legs keep stepping while it
    /// turns: a person does not spin on the spot.
    double turn_radius = 0.4;
    /// Turn of the model about Z so that it faces +X (rad).
    double model_yaw = 0.0;
    /// Scale of the model.
    double model_scale = 1.0;
    /// Time a clip takes to fade into the next one (s).
    double fade = 0.3;
    /// Clip of the model file for each role: idle, walk, run, talk, and any
    /// name a @c play action uses. A role without a clip plays nothing.
    std::map<std::string, std::string, std::less<>> clips;
    /// Named points of the body, in its frame (m): @c visitor.head. The
    /// @c head frame is there by default, at 93 % of the height.
    std::map<std::string, Vector3, std::less<>> frames;
};

// ****************************************************************************
/// Where an action goes or looks: a named place of the story, an entity, a
/// frame of an entity (@c robot.tool0, @c visitor.head) or a point.
// ****************************************************************************
struct StoryTarget
{
    /// Place, entity or @c entity.frame; empty when @ref point is set.
    std::string reference;
    /// A point of the floor (z ignored), in the world frame.
    std::optional<Vector3> point;
};

// ****************************************************************************
/// How fast a moving action goes.
// ****************************************************************************
enum class Gait : std::uint8_t
{
    Walk, ///< @c walk_speed, clip @c walk.
    Run,  ///< @c run_speed, clip @c run.
};

struct StoryAction;

// ****************************************************************************
/// One branch of a @c choose action, drawn with its weight.
// ****************************************************************************
struct StoryBranch
{
    /// Relative chance of this branch.
    double weight = 1.0;
    /// What the branch does.
    std::vector<StoryAction> actions;
};

// ****************************************************************************
/// One step of an event. The fields used depend on @ref type; the others
/// keep their default. The YAML keyword of each type is in the comment.
// ****************************************************************************
struct StoryAction
{
    // ------------------------------------------------------------------------
    /// What the action does.
    // ------------------------------------------------------------------------
    enum class Type : std::uint8_t
    {
        WalkTo,     ///< @c walk_to / @c run_to: go to a target, stop near it.
        FollowPath, ///< @c follow_path: go through points, in order.
        Follow,     ///< @c follow: keep a distance behind a target.
        TurnTo,     ///< @c turn_to: face a target or a heading.
        Wait,       ///< @c wait: stand still for a time.
        WaitUntil,  ///< @c wait_until: stand still until a condition holds.
        Say,        ///< @c say: speak a sentence.
        Play,       ///< @c play: play a clip (wave, sit, point).
        Expression, ///< @c expression: a face (smile, angry) for a view.
        Teleport,   ///< @c teleport: be somewhere at once.
        Show,       ///< @c show: be seen (true) or leave the world (false).
        Set,        ///< @c set: give a value to a story variable.
        Add,        ///< @c add: add to a story variable.
        Start,      ///< @c start: start other events now.
        Stop,       ///< @c stop: cancel other events.
        Choose,     ///< @c choose: play one branch, drawn by weight.
    };

    /// What the action does.
    Type type = Type::Wait;
    /// Line of the action in the YAML, for messages ("event 'arrive', action 2").
    std::string where;

    // --- Moves -------------------------------------------------------------
    /// Where to go, follow or look (one target; several for follow_path).
    std::vector<StoryTarget> targets;
    /// Walk or run.
    Gait gait = Gait::Walk;
    /// Speed (m/s), or nothing: the speed of the gait.
    std::optional<double> speed;
    /// @c walk_to stops this far from the target (m), body radius added;
    /// @c follow keeps this distance.
    double distance = 0.0;
    /// @c follow_path starts again at the first point; @c play loops.
    bool loop = false;
    /// @c turn_to a heading (rad) instead of a target.
    std::optional<double> heading;

    // --- Time --------------------------------------------------------------
    /// @c wait, @c play, @c say, @c expression: how long (s). For @c wait,
    /// with @ref duration_max, a time drawn in [duration, duration_max].
    double duration = 0.0;
    /// Upper bound of a drawn time; 0: not drawn.
    double duration_max = 0.0;
    /// @c wait_until, @c follow: the condition that ends it (PDDL notation).
    std::string condition;
    /// @c wait_until, @c walk_to, @c follow: gives up after this time (s),
    /// and the event fails; 0: never.
    double timeout = 0.0;

    // --- Speech, clips, variables -------------------------------------------
    /// @c say: id of the sentence, for @c (said who id); @c set/@c add: the
    /// variable; @c play: the clip role; @c expression: its name.
    std::string name;
    /// @c say: the sentence.
    std::string text;
    /// @c say: whom it speaks to (for the report; it turns to them).
    std::string to;
    /// @c set/@c add: the value; @c expression: the weight in [0, 1];
    /// @c show: 1 or 0.
    double value = 0.0;
    /// @c teleport: where, in the world frame (yaw used).
    Pose pose{ Vector3::ZERO, Quaternion{} };
    /// @c start / @c stop: the events.
    std::vector<std::string> events;
    /// @c choose: the branches.
    std::vector<StoryBranch> branches;
};

// ****************************************************************************
/// A list of actions an actor plays when a condition becomes true.
///
/// An event waits, starts on the rising edge of @ref start (false then true;
/// true at the first step counts), waits @ref delay, then plays its actions
/// one after the other. An actor plays one event at a time: a second one
/// waits for it, or cancels it when @ref interrupt is set.
///
/// @code{.yaml}
/// - id: greet
///   actor: visitor
///   start: (< (distance visitor robot) 2.0)
///   actions:
///     - turn_to: robot
///     - say: { id: hello, text: "Hello, I have a meeting at ten." }
/// @endcode
// ****************************************************************************
struct StoryEvent
{
    /// Unique id; conditions name it: @c (event_done greet).
    std::string id;
    /// Actor that plays it; empty for an event of the world only (set, add,
    /// start, stop, wait, wait_until, choose).
    std::string actor;
    /// Condition that starts it (PDDL notation); empty: at once.
    std::string start;
    /// Events that must be done before it starts.
    std::vector<std::string> after;
    /// Time between the start and the first action (s).
    double delay = 0.0;
    /// Times it may run; 0: no limit. It starts again on the next rising
    /// edge of @ref start.
    std::uint32_t repeat = 1U;
    /// Cancels the event the actor plays, instead of waiting for it.
    bool interrupt = false;
    /// Condition that cancels it while it runs; empty: never.
    std::string stop_when;
    /// What it does, in order.
    std::vector<StoryAction> actions;
};

// ****************************************************************************
/// What the actors do during a scenario (@c story: of a scenario file).
/// @code
/// robotik::StorySpec story;
/// story.places["desk"] = robotik::Vector3(1.0, 0.0, 0.0);
/// story.events.push_back({ .id = "arrive", .actor = "visitor",
///                          .actions = { { .type = robotik::StoryAction::Type::WalkTo,
///                                         .targets = { { .reference = "desk" } } } } });
/// @endcode
// ****************************************************************************
struct StorySpec
{
    /// Named points of the floor (m, world frame): where people go.
    std::map<std::string, Vector3, std::less<>> places;
    /// Story variables and their value at the start: @c (story_var name).
    std::map<std::string, double, std::less<>> variables;
    /// Every event, in file order.
    std::vector<StoryEvent> events;
};

} // namespace robotik
