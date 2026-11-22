// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "RobotIK/Task/RoboticCellDescription.hpp"

#include <algorithm>
#include <map>
#include <stdexcept>

namespace robotik
{

/// The robot flange and the tool link that lands on it (see Tools.md).
static constexpr char const* FLANGE = "flange";
static constexpr char const* TOOL_MOUNT = "tool_mount";

// ----------------------------------------------------------------------------
EntitySpec const* findEntity(WorldDescription const& p_world, std::string_view p_id)
{
    auto const found = std::ranges::find_if(p_world.entities,
                                            [p_id](EntitySpec const& p_entity)
                                            { return p_entity.id == p_id; });
    return found == p_world.entities.end() ? nullptr : &*found;
}

// ----------------------------------------------------------------------------
EntitySpec const& firstRobot(WorldDescription const& p_world)
{
    for (EntitySpec const& spec : p_world.entities)
    {
        if (spec.type == EntitySpec::Type::Robot)
        {
            return spec;
        }
    }
    // The readers refuse a world without robot: reaching here is a bug.
    throw std::logic_error("World '" + p_world.id + "' has no robot");
}

// ----------------------------------------------------------------------------
std::vector<EntitySpec const*> robotsOf(WorldDescription const& p_world)
{
    std::vector<EntitySpec const*> result;
    for (EntitySpec const& spec : p_world.entities)
    {
        if (spec.type == EntitySpec::Type::Robot)
        {
            result.push_back(&spec);
        }
    }
    return result;
}

// ----------------------------------------------------------------------------
EntitySpec const* mountedTool(WorldDescription const& p_world, std::string_view p_robot)
{
    std::string const robot = p_robot.empty() ? firstRobot(p_world).id : std::string(p_robot);
    for (EntitySpec const& spec : p_world.entities)
    {
        if (spec.type == EntitySpec::Type::Tool && spec.attach &&
            spec.attach->parent == robot)
        {
            return &spec;
        }
    }
    return nullptr;
}

// ----------------------------------------------------------------------------
Result<void> mountTool(WorldDescription& p_world, std::string_view p_id, std::string_view p_robot)
{
    std::string const robot_id =
        p_robot.empty() ? firstRobot(p_world).id : std::string(p_robot);
    EntitySpec const* owner = findEntity(p_world, robot_id);
    if (owner == nullptr || owner->type != EntitySpec::Type::Robot)
    {
        return failure("World '" + p_world.id + "' has no robot '" + robot_id + "'");
    }
    EntitySpec* chosen = nullptr;
    if (!p_id.empty())
    {
        for (EntitySpec& spec : p_world.entities)
        {
            if (spec.id == p_id && spec.type == EntitySpec::Type::Tool)
            {
                chosen = &spec;
            }
        }
        if (chosen == nullptr)
        {
            return failure("World '" + p_world.id + "' has no tool '" + std::string(p_id) + "'");
        }
    }
    for (EntitySpec& spec : p_world.entities)
    {
        if (spec.type == EntitySpec::Type::Tool && spec.attach &&
            (spec.attach->parent == robot_id || &spec == chosen))
        {
            spec.attach.reset();
        }
    }
    if (chosen != nullptr)
    {
        chosen->attach = Attachment{
            robot_id, FLANGE, TOOL_MOUNT, Pose{ Vector3::ZERO, Quaternion{} }
        };
    }
    return success();
}

// ----------------------------------------------------------------------------
char const* toString(Fidelity p_fidelity)
{
    switch (p_fidelity)
    {
        case Fidelity::Abstract:
            return "abstract";
        case Fidelity::Rigid:
            return "rigid";
        case Fidelity::Contact:
            return "contact";
    }
    return "?";
}

// ----------------------------------------------------------------------------
char const* toString(PlannerMethod p_method)
{
    switch (p_method)
    {
        case PlannerMethod::Search:
            return "search";
        case PlannerMethod::Backchaining:
            return "pabt";
    }
    return "?";
}

// ----------------------------------------------------------------------------
char const* toString(PerceptionFidelity p_fidelity)
{
    switch (p_fidelity)
    {
        case PerceptionFidelity::Sensed:
            return "sensed";
        case PerceptionFidelity::NoisyTruth:
            return "noisy_truth";
        case PerceptionFidelity::Truth:
            return "truth";
    }
    return "?";
}

// ----------------------------------------------------------------------------
PerceptionFidelity fidelityOf(PerceptionSpec const& p_perception,
                              std::string_view p_id,
                              bool p_blind)
{
    auto const found = p_perception.overrides.find(p_id);
    PerceptionFidelity const level =
        found == p_perception.overrides.end() ? p_perception.fidelity : found->second;
    return level == PerceptionFidelity::Sensed && p_blind ? p_perception.blind : level;
}

/// A description error prefixed with the place it was found.
static Failure fail(std::string const& p_where, std::string const& p_what)
{
    return failure(p_where + ": " + p_what);
}

/// Rules that hold for one entity alone.
static Result<void> validateEntity(EntitySpec const& p_spec,
                                   std::string const& p_where)
{
    switch (p_spec.type)
    {
        case EntitySpec::Type::Robot:
        case EntitySpec::Type::Tool:
            if (p_spec.model.empty())
            {
                return fail(p_where, "a robot or a tool needs a URDF model");
            }
            break;
        case EntitySpec::Type::Body:
            if (p_spec.shape && p_spec.shape->type != ShapeSpec::Type::Box)
            {
                return fail(p_where, "a body shape must be a box");
            }
            if (p_spec.shape && p_spec.physics == EntitySpec::Physics::Fixed)
            {
                return fail(p_where, "a primitive body must be free");
            }
            break;
        case EntitySpec::Type::Prop:
            if (!p_spec.shape || p_spec.physics != EntitySpec::Physics::None)
            {
                return fail(p_where, "a prop is a shape without physics");
            }
            break;
    }
    if (p_spec.type != EntitySpec::Type::Robot &&
        p_spec.type != EntitySpec::Type::Tool &&
        (!p_spec.sensors.empty() || !p_spec.home.empty()))
    {
        return fail(p_where,
                    "only robots and tools carry sensors or a home posture");
    }
    return success();
}

/// The attachment of one entity names a parent it can hang from.
static Result<void>
validateAttachment(WorldDescription const& p_world,
                   EntitySpec const& p_spec,
                   std::string const& p_where,
                   std::string const& p_entity,
                   std::map<std::string, int, std::less<>>& p_mounted)
{
    EntitySpec const* parent = findEntity(p_world, p_spec.attach->parent);
    if (parent == nullptr || parent == &p_spec)
    {
        return fail(p_entity,
                    "attach.parent '" + p_spec.attach->parent +
                        "' is not another entity");
    }
    if (p_spec.type == EntitySpec::Type::Tool)
    {
        if (parent->type != EntitySpec::Type::Robot ||
            p_spec.attach->parent_frame != FLANGE ||
            p_spec.attach->frame != TOOL_MOUNT)
        {
            return fail(p_entity,
                        "a tool attaches with { parent: <robot>.flange, "
                        "frame: tool_mount }");
        }
        if (++p_mounted[parent->id] > 1)
        {
            return fail(p_where,
                        "only one tool can be mounted on robot '" + parent->id +
                            "'");
        }
        return success();
    }
    if (p_spec.type == EntitySpec::Type::Robot)
    {
        return fail(p_entity, "a robot cannot be attached; give it a pose");
    }
    if (!p_spec.attach->frame.empty())
    {
        return fail(p_entity,
                    "only tools land a frame of their own on the parent");
    }
    if (parent->type == EntitySpec::Type::Tool)
    {
        return fail(p_entity, "cannot attach to a tool");
    }
    if (parent->attach && parent->type != EntitySpec::Type::Robot)
    {
        if (EntitySpec const* grand =
                findEntity(p_world, parent->attach->parent);
            grand != nullptr && grand->attach)
        {
            return fail(p_entity, "attachments nest two levels at most");
        }
    }
    return success();
}

// ----------------------------------------------------------------------------
Result<void> validate(WorldDescription const& p_world)
{
    std::string const where = p_world.id.empty() ? std::string("world")
                                                 : "world '" + p_world.id + "'";
    for (std::size_t i = 0; i < p_world.entities.size(); ++i)
    {
        EntitySpec const& spec = p_world.entities[i];
        if (spec.id.empty() || spec.id.find('.') != std::string::npos)
        {
            return fail(where, "every entity needs an id without '.'");
        }
        for (std::size_t j = 0; j < i; ++j)
        {
            if (p_world.entities[j].id == spec.id)
            {
                return fail(where,
                            "entity '" + spec.id + "' is declared twice");
            }
        }
        ROBOTIK_TRY(validateEntity(spec, where + ", entity '" + spec.id + "'"));
    }
    if (robotsOf(p_world).empty())
    {
        return fail(where, "an entity of type robot is required");
    }

    std::map<std::string, int, std::less<>> mounted;
    for (EntitySpec const& spec : p_world.entities)
    {
        std::string const entity = where + ", entity '" + spec.id + "'";
        if (spec.attach)
        {
            ROBOTIK_TRY(
                validateAttachment(p_world, spec, where, entity, mounted));
        }
        // What a tool works on is another entity of the world.
        if (!spec.touches.empty() && spec.type != EntitySpec::Type::Tool)
        {
            return fail(entity, "only a tool touches its workpieces");
        }
        for (std::string const& id : spec.touches)
        {
            if (findEntity(p_world, id) == nullptr || id == spec.id)
            {
                return fail(entity,
                            "touches '" + id +
                                "', which is not another entity");
            }
        }
    }

    // Robots share one resource manager: device names are unique in the world.
    std::map<std::string, std::string, std::less<>> devices;
    for (EntitySpec const& spec : p_world.entities)
    {
        std::vector<std::string> names;
        for (ActuatorSpec const& actuator : spec.actuators)
        {
            names.push_back(actuator.name);
        }
        for (SensorSpec const& sensor : spec.sensors)
        {
            names.push_back(sensor.name);
        }
        for (std::string const& name : names)
        {
            auto const [found, inserted] = devices.emplace(name, spec.id);
            if (!inserted && found->second != spec.id)
            {
                return fail(where,
                            "device '" + name + "' is declared by '" +
                                found->second + "' and by '" + spec.id +
                                "': names are unique in the world");
            }
        }
    }
    return success();
}

// ----------------------------------------------------------------------------
Result<void> validate(RoboticCellDescription const& p_description)
{
    ROBOTIK_TRY(validate(p_description.world));
    for (auto const& [id, level] : p_description.physics.overrides)
    {
        bool known = findEntity(p_description.world, id) != nullptr;
        for (EntitySpec const& entity : p_description.world.entities)
        {
            for (ActuatorSpec const& actuator : entity.actuators)
            {
                known = known || actuator.name == id;
            }
        }
        if (!known)
        {
            return failure("physics.overrides: '" + id +
                           "' is no entity nor actuator of the world");
        }
    }
    return success();
}

} // namespace robotik
