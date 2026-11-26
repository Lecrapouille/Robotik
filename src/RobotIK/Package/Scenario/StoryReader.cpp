// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "StoryReader.hpp"

#include <array>
#include <string>
#include <string_view>
#include <utility>

namespace robotik
{

/// The message of a reader, with where it happened.
static Failure fail(std::string const& p_where, std::string const& p_what)
{
    return failure(p_where + ": " + p_what);
}

/// A scalar read as a number, or a message.
static Result<double> numberOf(std::string const& p_where, ConfigNode const& p_node)
{
    std::optional<double> const value = p_node.asNumber();
    if (!p_node.isScalar() || !value)
    {
        return fail(p_where, "a number is expected");
    }
    return *value;
}

/// A scalar read as a boolean, or a message.
static Result<bool> booleanOf(std::string const& p_where, ConfigNode const& p_node)
{
    if (p_node.isScalar())
    {
        std::string const& text = p_node.scalar();
        if (text == "true" || text == "yes" || text == "on" || text == "1")
        {
            return true;
        }
        if (text == "false" || text == "no" || text == "off" || text == "0")
        {
            return false;
        }
    }
    return fail(p_where, "true or false is expected");
}

/// A list of numbers [x, y] or [x, y, z].
static Result<Vector3> pointOf(std::string const& p_where, ConfigNode const& p_node)
{
    std::vector<ConfigNode> const& items = p_node.items();
    if (!p_node.isSequence() || items.size() < 2U || items.size() > 3U)
    {
        return fail(p_where, "a point is [x, y] or [x, y, z]");
    }
    std::array<double, 3> xyz{ 0.0, 0.0, 0.0 };
    for (std::size_t i = 0; i < items.size(); ++i)
    {
        ROBOTIK_TRY_ASSIGN(xyz[i], numberOf(p_where, items[i]));
    }
    return Vector3(xyz[0], xyz[1], xyz[2]);
}

/// A target: a name (place, entity, entity.frame) or a point.
static Result<StoryTarget> targetOf(std::string const& p_where, ConfigNode const& p_node)
{
    StoryTarget target;
    if (p_node.isScalar() && !p_node.scalar().empty())
    {
        target.reference = p_node.scalar();
        return target;
    }
    if (p_node.isSequence())
    {
        ROBOTIK_TRY_ASSIGN(target.point, pointOf(p_where, p_node));
        return target;
    }
    return fail(p_where, "a target is a place, an entity, entity.frame or [x, y]");
}

/// A time: a number, or [min, max] drawn at each run.
static Result<void> durationOf(std::string const& p_where,
                               ConfigNode const& p_node,
                               StoryAction& p_action)
{
    if (p_node.isSequence())
    {
        std::vector<ConfigNode> const& items = p_node.items();
        if (items.size() != 2U)
        {
            return fail(p_where, "a time is a number or [min, max]");
        }
        ROBOTIK_TRY_ASSIGN(p_action.duration, numberOf(p_where, items[0]));
        ROBOTIK_TRY_ASSIGN(p_action.duration_max, numberOf(p_where, items[1]));
        if (p_action.duration_max < p_action.duration)
        {
            return fail(p_where, "[min, max] with max < min");
        }
    }
    else
    {
        ROBOTIK_TRY_ASSIGN(p_action.duration, numberOf(p_where, p_node));
    }
    if (p_action.duration < 0.0)
    {
        return fail(p_where, "a time is not negative");
    }
    return success();
}

/// The optional speed and gait of a move.
static Result<void> paceOf(std::string const& p_where,
                           ConfigNode const& p_node,
                           StoryAction& p_action)
{
    if (p_node.has("speed"))
    {
        double speed = 0.0;
        ROBOTIK_TRY_ASSIGN(speed, numberOf(p_where + ", speed", p_node.at("speed")));
        if (!(speed > 0.0))
        {
            return fail(p_where, "speed must be positive");
        }
        p_action.speed = speed;
    }
    std::string const gait = p_node.text("gait", p_action.gait == Gait::Run ? "run" : "walk");
    if (gait != "walk" && gait != "run")
    {
        return fail(p_where, "gait must be walk or run, not '" + gait + "'");
    }
    p_action.gait = gait == "run" ? Gait::Run : Gait::Walk;
    ROBOTIK_TRY_ASSIGN(p_action.timeout, p_node.number("timeout", 0.0));
    return success();
}

/// A name or a list of names (start, stop).
static Result<std::vector<std::string>> namesOf(std::string const& p_where,
                                                ConfigNode const& p_node)
{
    std::vector<std::string> names;
    if (p_node.isScalar())
    {
        names.push_back(p_node.scalar());
    }
    else if (p_node.isSequence())
    {
        for (ConfigNode const& item : p_node.items())
        {
            if (!item.isScalar())
            {
                return fail(p_where, "a list of names is expected");
            }
            names.push_back(item.scalar());
        }
    }
    if (names.empty())
    {
        return fail(p_where, "a name or a list of names is expected");
    }
    return names;
}

static Result<std::vector<StoryAction>> actionsOf(std::string const& p_where,
                                                  ConfigNode const& p_node,
                                                  std::string const& p_event);

/// The keywords of the actions, for the message of an unknown one.
static constexpr std::array<std::string_view, 18> ACTIONS = {
    "walk_to", "run_to", "follow_path", "follow", "turn_to", "wait",
    "wait_until", "say", "play", "expression", "teleport", "show",
    "set", "add", "start", "stop", "choose", "hide"
};

/// Reads one action: a map with one keyword (walk_to: desk).
static Result<StoryAction> actionOf(std::string const& p_where,
                                    ConfigNode const& p_item,
                                    std::string const& p_event,
                                    std::size_t p_index)
{
    if (!p_item.isMap() || p_item.entries().size() != 1U)
    {
        return fail(p_where, "an action is a map with one keyword, e.g. 'walk_to: desk'");
    }
    auto const& [keyword, value] = p_item.entries().front();
    StoryAction action;
    action.where = "action " + std::to_string(p_index + 1U) + " (" + keyword + ")";
    std::string const where = p_where + ", " + action.where;
    using Type = StoryAction::Type;

    if (keyword == "walk_to" || keyword == "run_to")
    {
        action.type = Type::WalkTo;
        action.gait = keyword == "run_to" ? Gait::Run : Gait::Walk;
        if (value.isMap())
        {
            ROBOTIK_TRY(value.checkKeys({ "target", "speed", "gait", "distance", "timeout" }, where));
            StoryTarget target;
            ROBOTIK_TRY_ASSIGN(target, targetOf(where, value.at("target")));
            action.targets.push_back(std::move(target));
            ROBOTIK_TRY(paceOf(where, value, action));
            ROBOTIK_TRY_ASSIGN(action.distance, value.number("distance", 0.0));
        }
        else
        {
            StoryTarget target;
            ROBOTIK_TRY_ASSIGN(target, targetOf(where, value));
            action.targets.push_back(std::move(target));
        }
    }
    else if (keyword == "follow_path")
    {
        action.type = Type::FollowPath;
        ConfigNode const* points = &value;
        if (value.isMap())
        {
            ROBOTIK_TRY(value.checkKeys({ "points", "loop", "speed", "gait", "timeout" }, where));
            points = &value.at("points");
            ROBOTIK_TRY(paceOf(where, value, action));
            ROBOTIK_TRY_ASSIGN(action.loop, value.flag("loop", false));
        }
        if (!points->isSequence() || points->items().empty())
        {
            return fail(where, "follow_path needs a list of points");
        }
        for (ConfigNode const& point : points->items())
        {
            StoryTarget target;
            ROBOTIK_TRY_ASSIGN(target, targetOf(where, point));
            action.targets.push_back(std::move(target));
        }
    }
    else if (keyword == "follow")
    {
        action.type = Type::Follow;
        action.distance = 1.0;
        if (value.isMap())
        {
            ROBOTIK_TRY(value.checkKeys({ "target", "distance", "until", "speed", "gait", "timeout" },
                                        where));
            StoryTarget target;
            ROBOTIK_TRY_ASSIGN(target, targetOf(where, value.at("target")));
            action.targets.push_back(std::move(target));
            ROBOTIK_TRY_ASSIGN(action.distance, value.number("distance", 1.0));
            action.condition = value.text("until");
            ROBOTIK_TRY(paceOf(where, value, action));
        }
        else
        {
            StoryTarget target;
            ROBOTIK_TRY_ASSIGN(target, targetOf(where, value));
            action.targets.push_back(std::move(target));
        }
    }
    else if (keyword == "turn_to")
    {
        action.type = Type::TurnTo;
        if (value.isMap())
        {
            ROBOTIK_TRY(value.checkKeys({ "target", "heading" }, where));
            if (value.has("heading") == value.has("target"))
            {
                return fail(where, "turn_to takes a target or a heading (rad), not both");
            }
            if (value.has("heading"))
            {
                double heading = 0.0;
                ROBOTIK_TRY_ASSIGN(heading, value.number("heading", 0.0));
                action.heading = heading;
            }
            else
            {
                StoryTarget target;
                ROBOTIK_TRY_ASSIGN(target, targetOf(where, value.at("target")));
                action.targets.push_back(std::move(target));
            }
        }
        else
        {
            StoryTarget target;
            ROBOTIK_TRY_ASSIGN(target, targetOf(where, value));
            action.targets.push_back(std::move(target));
        }
    }
    else if (keyword == "wait")
    {
        action.type = Type::Wait;
        ROBOTIK_TRY(durationOf(where, value, action));
    }
    else if (keyword == "wait_until")
    {
        action.type = Type::WaitUntil;
        if (value.isMap())
        {
            ROBOTIK_TRY(value.checkKeys({ "condition", "timeout" }, where));
            action.condition = value.text("condition");
            ROBOTIK_TRY_ASSIGN(action.timeout, value.number("timeout", 0.0));
        }
        else if (value.isScalar())
        {
            action.condition = value.scalar();
        }
        if (action.condition.empty())
        {
            return fail(where, "wait_until needs a condition, e.g. (> time 5)");
        }
    }
    else if (keyword == "say")
    {
        action.type = Type::Say;
        if (value.isMap())
        {
            ROBOTIK_TRY(value.checkKeys({ "id", "text", "to", "duration" }, where));
            action.name = value.text("id");
            action.text = value.text("text");
            action.to = value.text("to");
            ROBOTIK_TRY_ASSIGN(action.duration, value.number("duration", 0.0));
        }
        else if (value.isScalar())
        {
            action.text = value.scalar();
        }
        if (action.text.empty())
        {
            return fail(where, "say needs a text");
        }
        if (action.name.empty())
        {
            action.name = p_event + "_" + std::to_string(p_index + 1U);
        }
    }
    else if (keyword == "play")
    {
        action.type = Type::Play;
        if (value.isMap())
        {
            ROBOTIK_TRY(value.checkKeys({ "clip", "duration", "loop" }, where));
            action.name = value.text("clip");
            ROBOTIK_TRY_ASSIGN(action.duration, value.number("duration", 0.0));
            ROBOTIK_TRY_ASSIGN(action.loop, value.flag("loop", false));
        }
        else if (value.isScalar())
        {
            action.name = value.scalar();
        }
        if (action.name.empty())
        {
            return fail(where, "play needs a clip role, e.g. 'play: wave'");
        }
    }
    else if (keyword == "expression")
    {
        action.type = Type::Expression;
        action.value = 1.0;
        if (value.isMap())
        {
            ROBOTIK_TRY(value.checkKeys({ "name", "weight", "duration" }, where));
            action.name = value.text("name");
            ROBOTIK_TRY_ASSIGN(action.value, value.number("weight", 1.0));
            ROBOTIK_TRY_ASSIGN(action.duration, value.number("duration", 0.0));
        }
        else if (value.isScalar())
        {
            action.name = value.scalar();
        }
        if (action.name.empty() || action.value < 0.0 || action.value > 1.0)
        {
            return fail(where, "expression needs a name and a weight in [0, 1]");
        }
    }
    else if (keyword == "teleport")
    {
        action.type = Type::Teleport;
        if (!value.isMap())
        {
            return fail(where, "teleport takes { position: [x, y, z], rpy: [r, p, y] }");
        }
        ROBOTIK_TRY_ASSIGN(action.pose, value.asPose());
        ROBOTIK_TRY(value.checkKeys({ "position", "rpy", "quaternion" }, where));
    }
    else if (keyword == "show" || keyword == "hide")
    {
        action.type = Type::Show;
        bool shown = true;
        ROBOTIK_TRY_ASSIGN(shown, booleanOf(where, value));
        action.value = (shown == (keyword == "show")) ? 1.0 : 0.0;
    }
    else if (keyword == "set" || keyword == "add")
    {
        action.type = keyword == "set" ? Type::Set : Type::Add;
        if (!value.isMap() || value.entries().size() != 1U)
        {
            return fail(where, keyword + " takes one variable: { name: value }");
        }
        action.name = value.entries().front().first;
        ROBOTIK_TRY_ASSIGN(action.value, numberOf(where, value.entries().front().second));
    }
    else if (keyword == "start" || keyword == "stop")
    {
        action.type = keyword == "start" ? Type::Start : Type::Stop;
        ROBOTIK_TRY_ASSIGN(action.events, namesOf(where, value));
    }
    else if (keyword == "choose")
    {
        action.type = Type::Choose;
        if (!value.isSequence() || value.items().empty())
        {
            return fail(where, "choose takes a list of { weight, actions }");
        }
        for (std::size_t i = 0; i < value.items().size(); ++i)
        {
            ConfigNode const& item = value.items()[i];
            std::string const branch_where = where + ", branch " + std::to_string(i + 1U);
            ROBOTIK_TRY(item.checkKeys({ "weight", "actions" }, branch_where));
            StoryBranch branch;
            ROBOTIK_TRY_ASSIGN(branch.weight, item.number("weight", 1.0));
            if (!(branch.weight > 0.0))
            {
                return fail(branch_where, "weight must be positive");
            }
            ROBOTIK_TRY_ASSIGN(branch.actions,
                               actionsOf(branch_where, item.at("actions"), p_event));
            action.branches.push_back(std::move(branch));
        }
    }
    else
    {
        std::string known;
        for (std::string_view const name : ACTIONS)
        {
            known += (known.empty() ? "" : ", ") + std::string(name);
        }
        return fail(where, "unknown action '" + keyword + "' (known: " + known + ")");
    }
    if (action.timeout < 0.0 || action.distance < 0.0)
    {
        return fail(where, "timeout and distance are not negative");
    }
    return action;
}

/// Reads a list of actions.
static Result<std::vector<StoryAction>> actionsOf(std::string const& p_where,
                                                  ConfigNode const& p_node,
                                                  std::string const& p_event)
{
    if (!p_node.isSequence() || p_node.items().empty())
    {
        return fail(p_where, "actions must be a list of one action at least");
    }
    std::vector<StoryAction> actions;
    for (std::size_t i = 0; i < p_node.items().size(); ++i)
    {
        StoryAction action;
        ROBOTIK_TRY_ASSIGN(action, actionOf(p_where, p_node.items()[i], p_event, i));
        actions.push_back(std::move(action));
    }
    return actions;
}

/// Reads one event.
static Result<StoryEvent> eventOf(ConfigNode const& p_node, std::size_t p_index)
{
    StoryEvent event;
    event.id = p_node.text("id");
    std::string const where = "story, event '" +
                              (event.id.empty() ? std::to_string(p_index + 1U) : event.id) +
                              "'";
    ROBOTIK_TRY(p_node.checkKeys(
        { "id", "actor", "start", "after", "delay", "repeat", "interrupt", "stop_when", "actions" },
        where));
    if (event.id.empty() || event.id.find_first_of(" ().") != std::string::npos)
    {
        return fail(where, "every event needs an id without space, '(' or '.'");
    }
    event.actor = p_node.text("actor");
    event.start = p_node.text("start");
    event.stop_when = p_node.text("stop_when");
    if (p_node.has("after"))
    {
        ROBOTIK_TRY_ASSIGN(event.after, namesOf(where + ", after", p_node.at("after")));
    }
    ROBOTIK_TRY_ASSIGN(event.delay, p_node.number("delay", 0.0));
    if (event.delay < 0.0)
    {
        return fail(where, "delay is not negative");
    }
    if (p_node.text("repeat") == "forever")
    {
        event.repeat = 0U;
    }
    else
    {
        double repeat = 1.0;
        ROBOTIK_TRY_ASSIGN(repeat, p_node.number("repeat", 1.0));
        if (repeat < 1.0)
        {
            return fail(where, "repeat is a count of 1 at least, or forever");
        }
        event.repeat = static_cast<std::uint32_t>(repeat);
    }
    ROBOTIK_TRY_ASSIGN(event.interrupt, p_node.flag("interrupt", false));
    ROBOTIK_TRY_ASSIGN(event.actions, actionsOf(where, p_node.at("actions"), event.id));
    return event;
}

Result<StorySpec> readStory(ConfigNode const& p_node)
{
    StorySpec story;
    if (p_node.isNull())
    {
        return story;
    }
    ROBOTIK_TRY(p_node.checkKeys({ "places", "variables", "events" }, "story"));
    for (auto const& [name, value] : p_node.at("places").entries())
    {
        Vector3 point;
        ROBOTIK_TRY_ASSIGN(point, pointOf("story, place '" + name + "'", value));
        story.places[name] = point;
    }
    for (auto const& [name, value] : p_node.at("variables").entries())
    {
        double number = 0.0;
        ROBOTIK_TRY_ASSIGN(number, numberOf("story, variable '" + name + "'", value));
        story.variables[name] = number;
    }
    ConfigNode const& events = p_node.at("events");
    if (!events.isNull() && !events.isSequence())
    {
        return failure("story.events must be a list");
    }
    for (std::size_t i = 0; i < events.items().size(); ++i)
    {
        StoryEvent event;
        ROBOTIK_TRY_ASSIGN(event, eventOf(events.items()[i], i));
        story.events.push_back(std::move(event));
    }
    return story;
}

} // namespace robotik
