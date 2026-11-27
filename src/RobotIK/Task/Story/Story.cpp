// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "RobotIK/Task/Story/Story.hpp"

#include "RobotIK/Robot/SceneView.hpp"
#include "RobotIK/Task/RoboticCell.hpp"
#include "RobotIK/Task/RoboticCellDescription.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <iterator>
#include <numbers>
#include <sstream>
#include <stdexcept>

namespace robotik
{

/// The slowest an actor goes while it turns, as a part of its speed: never
/// zero, or the legs stop and the body slides round.
static constexpr double SLOWEST = 0.45;
/// Close enough to a point of a path (m).
static constexpr double PATH_TOLERANCE = 0.15;
/// Where walk_to stops near a place or a point (m), without a distance.
static constexpr double PLACE_STOP = 0.1;
/// Where walk_to stops near an entity (m), without a distance: a polite
/// distance from a person or a robot.
static constexpr double ENTITY_STOP = 0.8;
/// Facing a heading, within this angle (rad).
static constexpr double FACING = 0.03;
/// Words said in one second, for a say without a duration.
static constexpr double WORDS_PER_SECOND = 2.5;
/// A play without a duration lasts this long (s).
static constexpr double PLAY_LENGTH = 2.0;
/// Most actions one event may end in one step: a story of instant actions
/// that loops cannot hang the run.
static constexpr int MOST_ACTIONS_PER_STEP = 64;

/// An angle in [-pi, pi].
static double wrap(double p_angle)
{
    return std::remainder(p_angle, 2.0 * std::numbers::pi);
}

/// Horizontal distance between two points.
static double flat(Vector3 const& p_a, Vector3 const& p_b)
{
    return std::hypot(p_b.x - p_a.x, p_b.y - p_a.y);
}

/// Number of words of a sentence.
static std::size_t words(std::string const& p_text)
{
    std::istringstream stream(p_text);
    return static_cast<std::size_t>(
        std::distance(std::istream_iterator<std::string>(stream),
                      std::istream_iterator<std::string>()));
}

// ----------------------------------------------------------------------------
Story::Story(
    StorySpec const& p_spec,
    WorldDescription const& p_world,
    std::function<Result<Condition>(std::string_view)> const& p_compile)
    : m_spec(p_spec)
{
    for (EntitySpec const& entity : p_world.entities)
    {
        if (entity.type == EntitySpec::Type::Actor)
        {
            ActorState& actor = m_actors.emplace_back();
            actor.id = entity.id;
            actor.spec = &entity.actor;
        }
    }
    auto compile = [&](std::string const& p_text, std::string const& p_where)
    {
        Result<Condition> condition = p_compile(p_text);
        if (!condition)
        {
            throw std::invalid_argument(p_where + ": " + condition.error());
        }
        return std::move(condition).value();
    };
    for (StoryEvent const& spec : m_spec.events)
    {
        EventState& event = m_events.emplace_back();
        event.spec = &spec;
        event.actor = spec.actor.empty() ? NONE : actorIndex(spec.actor);
        std::string const where = "story, event '" + spec.id + "'";
        if (!spec.start.empty())
        {
            event.start = compile(spec.start, where + ", start");
        }
        if (!spec.stop_when.empty())
        {
            event.stop_when = compile(spec.stop_when, where + ", stop_when");
        }
        // Conditions of the actions, and of the actions of their branches.
        std::vector<StoryAction const*> actions;
        for (StoryAction const& action : spec.actions)
        {
            actions.push_back(&action);
        }
        for (std::size_t i = 0; i < actions.size(); ++i)
        {
            for (StoryBranch const& branch : actions[i]->branches)
            {
                for (StoryAction const& inner : branch.actions)
                {
                    actions.push_back(&inner);
                }
            }
            if (!actions[i]->condition.empty())
            {
                m_waits.emplace(actions[i],
                                compile(actions[i]->condition,
                                        where + ", " + actions[i]->where));
            }
        }
    }
}

// ----------------------------------------------------------------------------
void Story::reset(RoboticCell& p_cell, SceneView* p_view, Seed p_seed)
{
    m_random = Random(p_seed.derive("story"));
    m_variables.clear();
    for (auto const& [name, value] : m_spec.variables)
    {
        m_variables[name] = value;
    }
    m_utterances.clear();
    m_log.clear();
    m_now = p_cell.time();
    for (EventState& event : m_events)
    {
        event.phase = EventPhase::Waiting;
        // An event without a start condition starts at once.
        event.armed = !event.start.has_value();
        event.previous = false;
        event.starts = 0U;
        event.completions = 0U;
        event.failures = 0U;
        event.delay_left = Seconds(0.0);
        event.frames.clear();
        event.progress = {};
    }
    for (ActorState& actor : m_actors)
    {
        actor.heading = yaw(p_cell.requirePose(actor.id).rotation).value();
        actor.speed = 0.0;
        actor.shown = true;
        actor.role.clear();
        actor.loop = true;
        actor.owner = NONE;
        actor.speaking_until = Seconds(-1.0);
        actor.expression.clear();
        actor.expression_weight = 0.0;
        actor.expression_until = Seconds(-1.0);
        actor.face_set = false;
        if (p_view != nullptr)
        {
            compages::world::Entity entity = p_cell.entity(actor.id);
            p_view->shown(entity, true);
            p_view->say(entity, {}, 0.0);
        }
    }
    show(p_cell, p_view);
}

// ----------------------------------------------------------------------------
void Story::note(RoboticCell const& p_cell, std::string const& p_line)
{
    char time[32];
    std::snprintf(time, sizeof(time), "%.2f s ", p_cell.time().value());
    m_log.push_back(time + p_line);
}

// ----------------------------------------------------------------------------
std::size_t Story::eventIndex(std::string_view p_event) const
{
    for (std::size_t i = 0; i < m_events.size(); ++i)
    {
        if (m_events[i].spec->id == p_event)
        {
            return i;
        }
    }
    throw std::invalid_argument("the story has no event '" +
                                std::string(p_event) + "'");
}

std::size_t Story::actorIndex(std::string_view p_actor) const
{
    for (std::size_t i = 0; i < m_actors.size(); ++i)
    {
        if (m_actors[i].id == p_actor)
        {
            return i;
        }
    }
    throw std::invalid_argument("'" + std::string(p_actor) +
                                "' is no actor of the world");
}

bool Story::holds(Condition const& p_condition,
                  RoboticCell const& /*p_cell*/) const
{
    return evaluate(p_condition, m_metrics).holds;
}

Vector3 Story::where(RoboticCell const& p_cell,
                     StoryTarget const& p_target) const
{
    if (p_target.point)
    {
        return *p_target.point;
    }
    if (auto const place = m_spec.places.find(p_target.reference);
        place != m_spec.places.end())
    {
        return place->second;
    }
    return p_cell.requirePose(p_target.reference).position;
}

Vector3 Story::positionOf(RoboticCell& p_cell, ActorState const& p_actor) const
{
    auto const at = p_cell.entity(p_actor.id).position();
    return Vector3(static_cast<double>(at.x),
                   static_cast<double>(at.y),
                   static_cast<double>(at.z));
}

void Story::place(RoboticCell& p_cell,
                  ActorState& p_actor,
                  Vector3 const& p_at) const
{
    // The actors hang from the Z-up frame of the world: their local pose is
    // the world one.
    p_cell.entity(p_actor.id)
        .position(static_cast<float>(p_at.x),
                  static_cast<float>(p_at.y),
                  static_cast<float>(p_at.z))
        .rotation(compages::Quatf::fromAngleAxis(
            Radians(p_actor.heading), compages::Vector3f(0.0F, 0.0F, 1.0F)));
}

// ----------------------------------------------------------------------------
bool Story::turn(ActorState& p_actor, double p_heading, Seconds p_dt) const
{
    double const error = wrap(p_heading - p_actor.heading);
    double const step = p_actor.spec->turn_rate * p_dt.value();
    p_actor.heading = wrap(p_actor.heading + std::clamp(error, -step, step));
    return std::abs(error) <= std::max(FACING, step);
}

bool Story::walk(RoboticCell& p_cell,
                 ActorState& p_actor,
                 Vector3 const& p_goal,
                 double p_stop,
                 double p_speed,
                 Seconds p_dt) const
{
    Vector3 here = positionOf(p_cell, p_actor);
    double const distance = flat(here, p_goal);
    if (distance <= p_stop + 1e-6)
    {
        return true;
    }
    ActorSpec const& spec = *p_actor.spec;
    double const wanted = std::atan2(p_goal.y - here.y, p_goal.x - here.x);
    double const error = wrap(wanted - p_actor.heading);
    // Slower while it still turns a lot, but never still: on an arc, the
    // turn follows the steps.
    double const speed = p_speed * std::max(SLOWEST, std::cos(error));
    double rate = std::min(spec.turn_rate,
                           spec.turn_radius > 0.0 ? speed / spec.turn_radius
                                                  : spec.turn_rate);
    if (distance < 2.0 * spec.turn_radius)
    {
        // Near the goal, a tighter turn: else it circles round a goal
        // inside its arc.
        rate = spec.turn_rate;
    }
    double const turn_step = rate * p_dt.value();
    p_actor.heading =
        wrap(p_actor.heading + std::clamp(error, -turn_step, turn_step));
    double const step = std::min(speed * p_dt.value(), distance - p_stop);
    here.x += step * std::cos(p_actor.heading);
    here.y += step * std::sin(p_actor.heading);
    place(p_cell, p_actor, here);
    p_actor.speed = speed;
    return false;
}

// ----------------------------------------------------------------------------
void Story::fire(RoboticCell& p_cell, EventState& p_event)
{
    if (p_event.actor != NONE)
    {
        ActorState& actor = m_actors[p_event.actor];
        if (actor.owner != NONE && &m_events[actor.owner] != &p_event)
        {
            if (!p_event.spec->interrupt)
            {
                p_event.phase = EventPhase::Pending;
                return;
            }
            finish(p_cell,
                   m_events[actor.owner],
                   "cancelled by " + p_event.spec->id);
        }
        actor.owner = static_cast<std::size_t>(&p_event - m_events.data());
    }
    p_event.armed = false;
    ++p_event.starts;
    p_event.delay_left = Seconds(p_event.spec->delay);
    p_event.phase =
        p_event.spec->delay > 0.0 ? EventPhase::Delaying : EventPhase::Running;
    p_event.frames = { Frame{ &p_event.spec->actions, 0U } };
    p_event.progress = {};
    note(p_cell, p_event.spec->id + ": started");
}

void Story::finish(RoboticCell& p_cell,
                   EventState& p_event,
                   std::string const& p_how)
{
    if (p_how == "done")
    {
        ++p_event.completions;
    }
    else
    {
        ++p_event.failures;
    }
    if (p_event.actor != NONE &&
        m_actors[p_event.actor].owner ==
            static_cast<std::size_t>(&p_event - m_events.data()))
    {
        m_actors[p_event.actor].owner = NONE;
    }
    std::uint32_t const repeat = p_event.spec->repeat;
    p_event.phase = (repeat == 0U || p_event.starts < repeat)
                        ? EventPhase::Waiting
                        : EventPhase::Finished;
    // Without a start condition, a repeated event goes again at once.
    p_event.armed = !p_event.start.has_value();
    p_event.frames.clear();
    p_event.progress = {};
    note(p_cell, p_event.spec->id + ": " + p_how);
}

// ----------------------------------------------------------------------------
void Story::advance(RoboticCell& p_cell, EventState& p_event, Seconds p_dt)
{
    Seconds dt = p_dt;
    for (int i = 0; i < MOST_ACTIONS_PER_STEP; ++i)
    {
        if (p_event.frames.empty())
        {
            finish(p_cell, p_event, "done");
            return;
        }
        Frame& frame = p_event.frames.back();
        if (frame.next >= frame.actions->size())
        {
            p_event.frames.pop_back();
            continue;
        }
        StoryAction const& action = (*frame.actions)[frame.next];
        if (action.type == StoryAction::Type::Choose)
        {
            // Draw a branch by weight, then play it as a nested list.
            double total = 0.0;
            for (StoryBranch const& branch : action.branches)
            {
                total += branch.weight;
            }
            double draw = m_random.uniform(0.0, total);
            std::size_t chosen = action.branches.size() - 1U;
            for (std::size_t b = 0; b < action.branches.size(); ++b)
            {
                if (draw < action.branches[b].weight)
                {
                    chosen = b;
                    break;
                }
                draw -= action.branches[b].weight;
            }
            ++frame.next;
            note(p_cell,
                 p_event.spec->id + ": chose branch " +
                     std::to_string(chosen + 1U));
            p_event.frames.push_back(
                Frame{ &action.branches[chosen].actions, 0U });
            continue;
        }
        Outcome const outcome = play(p_cell, p_event, action, dt);
        if (p_event.phase != EventPhase::Running)
        {
            return; // the action stopped its own event
        }
        if (outcome == Outcome::Busy)
        {
            return;
        }
        if (outcome == Outcome::Failed)
        {
            finish(p_cell, p_event, "failed at " + action.where);
            return;
        }
        ++p_event.frames.back().next;
        p_event.progress = {};
        // The time of this step was used by the action that ended.
        dt = Seconds(0.0);
    }
}

// ----------------------------------------------------------------------------
Story::Outcome Story::play(RoboticCell& p_cell,
                           EventState& p_event,
                           StoryAction const& p_action,
                           Seconds p_dt)
{
    using Type = StoryAction::Type;
    Progress& progress = p_event.progress;
    bool const first = !progress.begun;
    progress.begun = true;
    progress.elapsed += p_dt;
    double const elapsed = progress.elapsed.value();
    bool const late = p_action.timeout > 0.0 && elapsed > p_action.timeout;
    ActorState* actor =
        p_event.actor == NONE ? nullptr : &m_actors[p_event.actor];

    auto speedOf = [&]() -> double
    {
        if (p_action.speed)
        {
            return *p_action.speed;
        }
        return p_action.gait == Gait::Run ? actor->spec->run_speed
                                          : actor->spec->walk_speed;
    };
    auto gaitRole = [&]()
    { return std::string(p_action.gait == Gait::Run ? "run" : "walk"); };

    switch (p_action.type)
    {
        case Type::WalkTo:
        {
            StoryTarget const& target = p_action.targets.front();
            bool const entity = !target.point && !isPlace(target.reference);
            double const stop = p_action.distance > 0.0
                                    ? p_action.distance
                                    : (entity ? ENTITY_STOP : PLACE_STOP);
            if (walk(p_cell,
                     *actor,
                     where(p_cell, target),
                     stop,
                     speedOf(),
                     p_dt))
            {
                return Outcome::Done;
            }
            actor->role = gaitRole();
            return late ? Outcome::Failed : Outcome::Busy;
        }
        case Type::FollowPath:
        {
            std::size_t const last = p_action.targets.size() - 1U;
            Vector3 const goal =
                where(p_cell, p_action.targets[progress.point]);
            if (walk(p_cell, *actor, goal, PATH_TOLERANCE, speedOf(), p_dt))
            {
                if (progress.point == last && !p_action.loop)
                {
                    return Outcome::Done;
                }
                progress.point =
                    progress.point == last ? 0U : progress.point + 1U;
            }
            actor->role = gaitRole();
            return late ? Outcome::Failed : Outcome::Busy;
        }
        case Type::Follow:
        {
            if (auto const until = m_waits.find(&p_action);
                until != m_waits.end() && holds(until->second, p_cell))
            {
                return Outcome::Done;
            }
            if (late)
            {
                return Outcome::Done;
            }
            Vector3 const goal = where(p_cell, p_action.targets.front());
            Vector3 const here = positionOf(p_cell, *actor);
            // It goes when the target is a margin farther than the
            // distance, and stops at the distance: it does not start and
            // stop at each step. progress.point says it walks.
            if (flat(here, goal) > p_action.distance + 0.25)
            {
                progress.point = 1U;
            }
            if (progress.point == 1U)
            {
                if (!walk(p_cell,
                          *actor,
                          goal,
                          p_action.distance,
                          speedOf(),
                          p_dt))
                {
                    actor->role = gaitRole();
                    return Outcome::Busy;
                }
                progress.point = 0U;
            }
            turn(*actor, std::atan2(goal.y - here.y, goal.x - here.x), p_dt);
            place(p_cell, *actor, here);
            return Outcome::Busy;
        }
        case Type::TurnTo:
        {
            double heading = p_action.heading.value_or(0.0);
            if (!p_action.heading)
            {
                Vector3 const goal = where(p_cell, p_action.targets.front());
                Vector3 const here = positionOf(p_cell, *actor);
                heading = std::atan2(goal.y - here.y, goal.x - here.x);
            }
            bool const facing = turn(*actor, heading, p_dt);
            place(p_cell, *actor, positionOf(p_cell, *actor));
            return facing ? Outcome::Done : Outcome::Busy;
        }
        case Type::Wait:
        {
            if (first)
            {
                progress.length = p_action.duration_max > 0.0
                                      ? m_random.uniform(p_action.duration,
                                                         p_action.duration_max)
                                      : p_action.duration;
            }
            return elapsed >= progress.length ? Outcome::Done : Outcome::Busy;
        }
        case Type::WaitUntil:
        {
            if (holds(m_waits.at(&p_action), p_cell))
            {
                return Outcome::Done;
            }
            return late ? Outcome::Failed : Outcome::Busy;
        }
        case Type::Say:
        {
            if (first)
            {
                progress.length =
                    p_action.duration > 0.0
                        ? p_action.duration
                        : std::max(1.0,
                                   static_cast<double>(words(p_action.text)) /
                                       WORDS_PER_SECOND);
                m_utterances.push_back({ p_cell.time(),
                                         actor->id,
                                         p_action.name,
                                         p_action.text,
                                         p_action.to });
                actor->speaking_until =
                    p_cell.time() + Seconds(progress.length);
                note(p_cell,
                     actor->id + " says '" + p_action.text + "'" +
                         (p_action.to.empty() ? "" : " to " + p_action.to));
                if (m_view != nullptr)
                {
                    m_view->say(p_cell.entity(actor->id),
                                p_action.text,
                                actor->spec->height + 0.25);
                }
            }
            if (!p_action.to.empty())
            {
                Vector3 const goal =
                    where(p_cell, StoryTarget{ p_action.to, std::nullopt });
                Vector3 const here = positionOf(p_cell, *actor);
                turn(
                    *actor, std::atan2(goal.y - here.y, goal.x - here.x), p_dt);
                place(p_cell, *actor, here);
            }
            actor->role = "talk";
            return elapsed >= progress.length ? Outcome::Done : Outcome::Busy;
        }
        case Type::Play:
        {
            actor->role = p_action.name;
            actor->loop = p_action.loop;
            double const length =
                p_action.duration > 0.0 ? p_action.duration : PLAY_LENGTH;
            return elapsed >= length ? Outcome::Done : Outcome::Busy;
        }
        case Type::Expression:
        {
            actor->expression = p_action.name;
            actor->expression_weight = p_action.value;
            actor->expression_until =
                p_action.duration > 0.0
                    ? p_cell.time() + Seconds(p_action.duration)
                    : Seconds(-1.0);
            actor->face_set = false;
            return Outcome::Done;
        }
        case Type::Teleport:
        {
            actor->heading = yaw(p_action.pose.rotation).value();
            place(p_cell, *actor, p_action.pose.position);
            return Outcome::Done;
        }
        case Type::Show:
        {
            actor->shown = p_action.value > 0.0;
            if (m_view != nullptr)
            {
                m_view->shown(p_cell.entity(actor->id), actor->shown);
            }
            note(p_cell, actor->id + (actor->shown ? " enters" : " leaves"));
            return Outcome::Done;
        }
        case Type::Set:
            m_variables[p_action.name] = p_action.value;
            return Outcome::Done;
        case Type::Add:
            m_variables[p_action.name] += p_action.value;
            return Outcome::Done;
        case Type::Start:
            for (std::string const& id : p_action.events)
            {
                EventState& event = m_events[eventIndex(id)];
                if (event.phase == EventPhase::Waiting)
                {
                    event.armed = true;
                    event.forced = true;
                }
            }
            return Outcome::Done;
        case Type::Stop:
            for (std::string const& id : p_action.events)
            {
                EventState& event = m_events[eventIndex(id)];
                if (event.phase == EventPhase::Running ||
                    event.phase == EventPhase::Delaying ||
                    event.phase == EventPhase::Pending)
                {
                    finish(p_cell, event, "cancelled by " + p_event.spec->id);
                }
                event.armed = false;
            }
            return Outcome::Done;
        case Type::Choose:
            break; // played by advance
    }
    return Outcome::Done;
}

// ----------------------------------------------------------------------------
void Story::step(RoboticCell& p_cell, SceneView* p_view, Seconds p_dt)
{
    m_view = p_view;
    m_now = p_cell.time();
    m_metrics = p_cell.metrics();
    for (ActorState& actor : m_actors)
    {
        actor.speed = 0.0;
        actor.role.clear();
        actor.loop = true;
    }
    for (EventState& event : m_events)
    {
        if (event.phase == EventPhase::Finished)
        {
            continue;
        }
        if (event.phase == EventPhase::Waiting)
        {
            bool const now = !event.start || holds(*event.start, p_cell);
            if (now && !event.previous)
            {
                event.armed = true;
            }
            event.previous = now;
            bool after = true;
            for (std::string const& id : event.spec->after)
            {
                after = after && m_events[eventIndex(id)].completions > 0U;
            }
            if (event.armed && (after || event.forced))
            {
                event.forced = false;
                fire(p_cell, event);
            }
        }
        else if (event.phase == EventPhase::Pending)
        {
            fire(p_cell, event);
        }
        if (event.phase == EventPhase::Delaying)
        {
            event.delay_left -= p_dt;
            if (event.delay_left.value() > 0.0)
            {
                continue;
            }
            event.phase = EventPhase::Running;
        }
        if (event.phase == EventPhase::Running)
        {
            if (event.stop_when && holds(*event.stop_when, p_cell))
            {
                finish(p_cell, event, "cancelled by its stop_when");
                continue;
            }
            advance(p_cell, event, p_dt);
        }
    }
    show(p_cell, p_view);
}

// ----------------------------------------------------------------------------
void Story::show(RoboticCell& p_cell, SceneView* p_view)
{
    for (ActorState& actor : m_actors)
    {
        bool const speaking = p_cell.time() < actor.speaking_until;
        if (actor.speaking_until.value() >= 0.0 && !speaking)
        {
            actor.speaking_until = Seconds(-1.0);
            if (p_view != nullptr)
            {
                p_view->say(p_cell.entity(actor.id), {}, 0.0);
            }
        }
        if (actor.expression_until.value() >= 0.0 &&
            p_cell.time() >= actor.expression_until)
        {
            actor.expression_until = Seconds(-1.0);
            actor.expression_weight = 0.0;
            actor.face_set = false;
        }
        if (p_view == nullptr)
        {
            continue;
        }
        compages::world::Entity entity = p_cell.entity(actor.id);
        std::string role = actor.role;
        if (role.empty())
        {
            role = speaking ? "talk" : "idle";
        }
        // A role without a clip in the actor plays the clip of that name.
        auto const clip = actor.spec->clips.find(role);
        std::string const name =
            clip != actor.spec->clips.end() ? clip->second : role;
        bool const moving = role == "walk" || role == "run";
        p_view->animate(entity, name, Seconds(actor.spec->fade), actor.loop,
                        moving ? actor.speed : 0.0);
        if (!actor.face_set && !actor.expression.empty())
        {
            p_view->expression(
                entity, actor.expression, actor.expression_weight);
            actor.face_set = true;
        }
    }
}

// --- Answers of the queries -------------------------------------------------

EventPhase Story::requirePhase(std::string_view p_event) const
{
    return m_events[eventIndex(p_event)].phase;
}

std::uint32_t Story::requireStarts(std::string_view p_event) const
{
    return m_events[eventIndex(p_event)].starts;
}

std::uint32_t Story::requireCompletions(std::string_view p_event) const
{
    return m_events[eventIndex(p_event)].completions;
}

std::uint32_t Story::requireFailures(std::string_view p_event) const
{
    return m_events[eventIndex(p_event)].failures;
}

double Story::requireVariable(std::string_view p_name) const
{
    auto const found = m_variables.find(p_name);
    if (found == m_variables.end())
    {
        throw std::invalid_argument("the story has no variable '" +
                                    std::string(p_name) + "'");
    }
    return found->second;
}

bool Story::said(std::string_view p_speaker, std::string_view p_id) const
{
    return std::ranges::any_of(
        m_utterances,
        [&](Utterance const& p_utterance)
        { return p_utterance.speaker == p_speaker && p_utterance.id == p_id; });
}

double Story::requireSpeed(std::string_view p_actor) const
{
    return m_actors[actorIndex(p_actor)].speed;
}

bool Story::requireShown(std::string_view p_actor) const
{
    return m_actors[actorIndex(p_actor)].shown;
}

bool Story::requireSpeaking(std::string_view p_actor) const
{
    return m_now < m_actors[actorIndex(p_actor)].speaking_until;
}

double Story::requireHeading(std::string_view p_actor) const
{
    return m_actors[actorIndex(p_actor)].heading;
}

bool Story::isActor(std::string_view p_id) const
{
    return std::ranges::any_of(m_actors,
                               [&](ActorState const& p_actor)
                               { return p_actor.id == p_id; });
}

bool Story::isPlace(std::string_view p_name) const
{
    return m_spec.places.contains(p_name);
}

double Story::expression(std::string_view p_actor,
                         std::string_view p_name) const
{
    ActorState const& actor = m_actors[actorIndex(p_actor)];
    return actor.expression == p_name ? actor.expression_weight : 0.0;
}

Vector3 Story::requirePlace(std::string_view p_name) const
{
    auto const found = m_spec.places.find(p_name);
    if (found == m_spec.places.end())
    {
        throw std::invalid_argument("the story has no place '" +
                                    std::string(p_name) + "'");
    }
    return found->second;
}

} // namespace robotik
