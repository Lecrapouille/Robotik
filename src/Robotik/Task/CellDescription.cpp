// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.


#include "Robotik/Task/CellDescription.hpp"

#include <algorithm>
#include <cmath>
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

/// Thickness of the stop at the end of a belt, and its height above the belt (m).
static constexpr double STOP_THICKNESS = 0.01;
static constexpr double STOP_HEIGHT = 0.02;

/// The stop at the end of a belt: a fixed box just past its end, across it.
static EntitySpec beltStop(EntitySpec const& p_belt,
                           ActuatorSpec const& p_conveyor)
{
    Vector3 const& size = p_belt.shape->size;
    double const length =
        std::hypot(p_conveyor.direction.x, p_conveyor.direction.y);
    double const dx = p_conveyor.direction.x / length;
    double const dy = p_conveyor.direction.y / length;
    // Half length of the belt along the direction, its width across it.
    double const half = 0.5 * (std::abs(dx) * size.x + std::abs(dy) * size.y);
    double const width = std::abs(dy) * size.x + std::abs(dx) * size.y;

    EntitySpec stop;
    stop.id = p_belt.id + "_stop";
    stop.type = EntitySpec::Type::Body;
    stop.physics = EntitySpec::Physics::Fixed;
    stop.friction = p_belt.friction;
    ShapeSpec shape = *p_belt.shape;
    shape.size = Vector3(STOP_THICKNESS, width, size.z + STOP_HEIGHT);
    shape.color = { 0.75F, 0.55F, 0.1F };
    stop.shape = shape;
    Pose const local{ Vector3(dx * (half + 0.5 * STOP_THICKNESS),
                              dy * (half + 0.5 * STOP_THICKNESS),
                              0.5 * STOP_HEIGHT),
                      Quaternion::fromAngleAxis(Radians(std::atan2(dy, dx)),
                                                Vector3(0.0, 0.0, 1.0)) };
    stop.pose = p_belt.pose * local;
    return stop;
}

// ----------------------------------------------------------------------------
Result<void> addBeltStops(WorldDescription& p_world)
{
    std::vector<EntitySpec> stops;
    for (EntitySpec const& spec : p_world.entities)
    {
        for (ActuatorSpec const& actuator : spec.actuators)
        {
            if (actuator.type != ActuatorSpec::Type::Conveyor ||
                !actuator.stop || !spec.shape)
            {
                continue;
            }
            if (spec.attach)
            {
                return failure("Entity '" + spec.id +
                               "': a belt with a stop is placed by its pose, "
                               "not attached");
            }
            stops.push_back(beltStop(spec, actuator));
            if (findEntity(p_world, stops.back().id) != nullptr)
            {
                return failure("Entity '" + stops.back().id +
                               "' is the stop of a belt: choose another id");
            }
        }
    }
    for (EntitySpec& stop : stops)
    {
        p_world.entities.push_back(std::move(stop));
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

} // namespace robotik
