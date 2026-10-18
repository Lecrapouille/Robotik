// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file SimulationTruth.cpp
/// @brief Ground-truth queries of @ref Simulation: what the assertions
/// measure. World frame, metres and radians.

#include "Robotik/Runtime/Simulation.hpp"
#include "ShapeOf.hpp"

#include "Robotik/Backends/PinocchioBackend.hpp"
#include "Robotik/ECS/ObjectComponents.hpp"
#include "Robotik/Robot/Actuators.hpp"
#include "Robotik/Scene/ContainerBounds.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

/// Vertical gap tolerated between two props said to rest on each other (m).
#define ON_TOLERANCE_M 0.005
/// Joint limit overshoot tolerated by joint_limits_respected (rad or m).
#define JOINT_LIMIT_TOLERANCE 1e-3

namespace robotik
{

/// Drops leading and trailing spaces and tabs.
static std::string_view trim(std::string_view p_text)
{
    std::size_t const first = p_text.find_first_not_of(" \t");
    if (first == std::string_view::npos)
    {
        return {};
    }
    std::size_t const last = p_text.find_last_not_of(" \t");
    return p_text.substr(first, last - first + 1u);
}

/// "name(a, b)" -> name and its arguments (commas inside parentheses kept,
/// quotes stripped). Nothing when @p_term is not a call.
static std::optional<std::pair<std::string, std::vector<std::string>>> parseCall(std::string_view p_term)
{
    std::string_view const term = trim(p_term);
    std::size_t const open = term.find('(');
    if (open == std::string_view::npos || open == 0u || term.back() != ')')
    {
        return std::nullopt;
    }
    std::pair<std::string, std::vector<std::string>> call;
    call.first = std::string(trim(term.substr(0, open)));
    std::string_view const inside = term.substr(open + 1u, term.size() - open - 2u);
    int depth = 0;
    std::size_t start = 0;
    auto push = [&](std::size_t p_end)
    {
        std::string_view argument = trim(inside.substr(start, p_end - start));
        if (argument.size() >= 2u && argument.front() == '"' && argument.back() == '"')
        {
            argument = argument.substr(1u, argument.size() - 2u);
        }
        if (!argument.empty())
        {
            call.second.emplace_back(argument);
        }
    };
    for (std::size_t i = 0; i < inside.size(); ++i)
    {
        if (inside[i] == '(')
        {
            ++depth;
        }
        else if (inside[i] == ')')
        {
            --depth;
        }
        else if (inside[i] == ',' && depth == 0)
        {
            push(i);
            start = i + 1u;
        }
    }
    push(inside.size());
    return call;
}

/// "easel.canvas_mount" -> { "easel", "canvas_mount" }.
static std::pair<std::string_view, std::string_view> splitReference(std::string_view p_reference)
{
    std::size_t const dot = p_reference.find('.');
    if (dot == std::string_view::npos)
    {
        return { p_reference, {} };
    }
    return { p_reference.substr(0, dot), p_reference.substr(dot + 1u) };
}

/// Angle in radians between two orientations.
static double angleBetween(Quaternion const& p_a, Quaternion const& p_b)
{
    double const dot = std::abs(p_a.w * p_b.w + p_a.x * p_b.x + p_a.y * p_b.y + p_a.z * p_b.z);
    return 2.0 * std::acos(std::min(1.0, dot));
}

/// Half the extents of an entity shape, or zero when it has none.
static Vector3 halfSize(EntitySpec const& p_spec)
{
    return p_spec.shape ? Vector3(0.5 * p_spec.shape->size.x,
                                  0.5 * p_spec.shape->size.y,
                                  0.5 * p_spec.shape->size.z)
                        : zero3();
}

/// Rejects a truth query whose argument count is out of range.
static void arity(std::string const& p_name,
           std::vector<std::string> const& p_args,
           std::size_t p_min,
           std::size_t p_max)
{
    if (p_args.size() < p_min || p_args.size() > p_max)
    {
        throw std::invalid_argument(p_name + "() takes " + std::to_string(p_min) +
                                    (p_min == p_max ? "" : " to " + std::to_string(p_max)) +
                                    " argument(s)");
    }
}

Simulation::Record const* Simulation::record(std::string_view p_id) const
{
    for (Record const& record : m_records)
    {
        if (record.spec->id == p_id)
        {
            return &record;
        }
    }
    return nullptr;
}

Simulation::Record const& Simulation::require(std::string_view p_id) const
{
    if (Record const* found = record(p_id))
    {
        return *found;
    }
    if (EntitySpec const* spec = m_scenario.world.find(p_id);
        spec != nullptr && spec->type == EntitySpec::Type::Tool)
    {
        throw std::invalid_argument("tool '" + std::string(p_id) + "' is not mounted");
    }
    throw std::invalid_argument("unknown entity '" + std::string(p_id) + "'");
}

compages::world::Entity Simulation::decorate(ecs::Shape const& p_object)
{
    compages::world::Entity entity = m_world.entity(p_object.name);
    entity.parent(m_frame).set(p_object).set(ecs::Decoration{});
    if (m_view != nullptr)
    {
        m_view->object(entity, p_object);
    }
    return entity;
}

compages::world::Entity Simulation::entity(std::string_view p_id) const
{
    Record const* found = record(p_id);
    return found != nullptr ? found->entity : compages::world::Entity{};
}

Pose Simulation::framePose(Record const& p_record, std::string_view p_frame) const
{
    EntitySpec const& spec = *p_record.spec;
    std::string const frame(p_frame);
    switch (spec.type)
    {
        case EntitySpec::Type::Robot:
            return ownerOf(spec).worldPose(frame);
        case EntitySpec::Type::Tool:
            // A mounted tool is part of the robot chain; its tool_mount lands
            // on the flange (Pinocchio merges the two).
            return ownerOf(spec).worldPose(frame.empty() ? p_record.spec->attach->parent_frame
                                                         : frame);
        case EntitySpec::Type::Prop:
        {
            if (!frame.empty())
            {
                throw std::invalid_argument("prop '" + spec.id + "' has no frame '" + frame + "'");
            }
            auto const at = p_record.entity.position();
            return { Vector3(at.x, at.y, at.z), p_record.nominal.rotation };
        }
        case EntitySpec::Type::Body:
            break;
    }
    RobotBackend const* backend = m_robot->backend();
    std::optional<Pose> const root =
        backend != nullptr ? backend->bodyPose(p_record.body) : std::nullopt;
    Pose const base = root ? *root : p_record.nominal;
    if (frame.empty())
    {
        return base;
    }
    if (p_record.frames == nullptr || !p_record.frames->hasFrame(frame))
    {
        throw std::invalid_argument("'" + spec.id + "' has no frame '" + frame + "'");
    }
    return base * p_record.frames->framePose(frame);
}

Pose Simulation::pose(std::string_view p_reference) const
{
    auto const [id, frame] = splitReference(p_reference);
    return framePose(require(id), frame);
}

Pose Simulation::initialPose(std::string_view p_id) const
{
    return require(p_id).initial;
}

Pose Simulation::described(std::string_view p_reference) const
{
    auto const [id, frame] = splitReference(p_reference);
    Record const& record = require(id);
    switch (record.spec->type)
    {
        case EntitySpec::Type::Robot:
        case EntitySpec::Type::Tool:
            // The robot knows its own body.
            return framePose(record, frame);
        case EntitySpec::Type::Prop:
            if (!frame.empty())
            {
                throw std::invalid_argument("prop '" + record.spec->id + "' has no frame '" +
                                            std::string(frame) + "'");
            }
            return record.nominal;
        case EntitySpec::Type::Body:
            break;
    }
    if (frame.empty())
    {
        return record.nominal;
    }
    std::string const link(frame);
    if (record.frames == nullptr || !record.frames->hasFrame(link))
    {
        throw std::invalid_argument("'" + record.spec->id + "' has no frame '" + link + "'");
    }
    return record.nominal * record.frames->framePose(link);
}

std::vector<Pose> Simulation::waypoints(std::string_view p_name) const
{
    auto const found = m_scenario.trajectories.find(p_name);
    if (found == m_scenario.trajectories.end())
    {
        throw std::invalid_argument("the scenario has no trajectory '" + std::string(p_name) + "'");
    }
    Trajectory const& trajectory = found->second;
    if (trajectory.type != Trajectory::Type::Cartesian)
    {
        throw std::invalid_argument("trajectory '" + std::string(p_name) + "' is not cartesian");
    }
    Pose const frame = trajectory.frame.empty() ? Pose{ zero3(), Quaternion{} }
                                                : described(trajectory.frame);
    Pose const to_base = m_robot->base().pose.inverse() * frame;
    std::vector<Pose> result;
    result.reserve(trajectory.poses.size());
    for (Pose const& point : trajectory.poses)
    {
        result.push_back(to_base * point);
    }
    return result;
}

int Simulation::contactsOf(Record const& p_record, std::vector<Contact> const& p_contacts) const
{
    bool const robot_side = p_record.spec->type == EntitySpec::Type::Robot ||
                            p_record.spec->type == EntitySpec::Type::Tool;
    // The robot answers for its mounted tool: a tool hitting the world is a
    // robot collision too.
    auto mine = [&](std::string const& p_body)
    {
        if (p_record.links.contains(p_body))
        {
            return true;
        }
        return p_record.spec->type == EntitySpec::Type::Robot &&
               std::any_of(m_records.begin(), m_records.end(),
                           [&](Record const& p_peer)
                           { return p_peer.spec->type == EntitySpec::Type::Tool && p_peer.links.contains(p_body); });
    };
    int count = 0;
    for (Contact const& contact : p_contacts)
    {
        bool const first = mine(contact.first);
        bool const second = mine(contact.second);
        // Self contacts and the floor do not count.
        if (first == second || contact.first == Contact::GROUND ||
            contact.second == Contact::GROUND)
        {
            continue;
        }
        if (robot_side)
        {
            // The mounted tool and the robot are one assembly.
            std::string const& other = first ? contact.second : contact.first;
            bool const assembly = std::any_of(
                m_records.begin(),
                m_records.end(),
                [&](Record const& p_peer)
                {
                    return (p_peer.spec->type == EntitySpec::Type::Robot ||
                            p_peer.spec->type == EntitySpec::Type::Tool) &&
                           p_peer.links.contains(other);
                });
            if (assembly)
            {
                continue;
            }
        }
        ++count;
    }
    return count;
}

Metrics Simulation::metrics() const
{
    Metrics metrics;
    metrics.set("time", time().value());
    metrics.set("controller.success", m_status == bt::Status::SUCCESS ? 1.0 : 0.0);
    if (m_executor)
    {
        metrics.set("planner.plans", static_cast<double>(m_executor->history().size()));
        metrics.set("planner.replans", static_cast<double>(m_executor->replans()));
        metrics.set("planner.steps", static_cast<double>(m_executor->plan().size()));
        metrics.set("planner.done", static_cast<double>(m_executor->done()));
    }
    if (m_mission != nullptr)
    {
        m_mission->measure(*this, metrics);
    }
    return metrics;
}

Evaluation Simulation::evaluate(std::string_view p_condition) const
{
    Metrics const table = metrics();
    return robotik::evaluate(p_condition,
                            table,
                            [this](std::string_view p_term) { return query(p_term); });
}

std::optional<double> Simulation::query(std::string_view p_term) const
{
    if (auto const call = parseCall(p_term))
    {
        return this->call(call->first, call->second);
    }
    return metrics().get(trim(p_term));
}

std::optional<double> Simulation::call(std::string const& p_name,
                                       std::vector<std::string> const& p_args) const
{
    auto boolean = [](bool p_value) { return p_value ? 1.0 : 0.0; };

    if (p_name == "pose_error" || p_name == "orientation_error")
    {
        arity(p_name, p_args, 1u, 1u);
        Record const& record = require(p_args[0]);
        Pose const now = framePose(record, {});
        if (p_name == "pose_error")
        {
            return norm(now.position - record.initial.position);
        }
        return angleBetween(now.rotation, record.initial.rotation);
    }
    if (p_name == "distance")
    {
        arity(p_name, p_args, 2u, 2u);
        return norm(pose(p_args[0]).position - pose(p_args[1]).position);
    }
    if (p_name == "inside")
    {
        arity(p_name, p_args, 2u, 2u);
        Record const& object = require(p_args[0]);
        Record const& container = require(p_args[1]);
        if (!object.spec->shape || !container.spec->shape)
        {
            throw std::invalid_argument("inside() compares two primitive shapes");
        }
        Vector3 const at = framePose(object, {}).position;
        Vector3 const center = framePose(container, {}).position;
        if (container.spec->shape->type == ShapeSpec::Type::Container)
        {
            return boolean(scene::restsInside(
                shapeOf(*container.spec), center, at, halfSize(*object.spec)));
        }
        Vector3 const half = halfSize(*container.spec);
        return boolean(std::abs(at.x - center.x) < half.x && std::abs(at.y - center.y) < half.y &&
                       std::abs(at.z - center.z) < half.z);
    }
    if (p_name == "touching" || p_name == "on")
    {
        arity(p_name, p_args, 2u, 2u);
        Record const& upper = require(p_args[0]);
        Record const& lower = require(p_args[1]);
        RobotBackend const* backend = m_robot->backend();
        if (!upper.links.empty() && !lower.links.empty() && backend != nullptr)
        {
            // Bodies of the physics backend: a contact between them, below the
            // upper one for "on".
            double const height = framePose(upper, {}).position.z;
            for (Contact const& contact : backend->contactList())
            {
                bool const between =
                    (upper.links.contains(contact.first) && lower.links.contains(contact.second)) ||
                    (upper.links.contains(contact.second) && lower.links.contains(contact.first));
                if (between && (p_name == "touching" || contact.position.z < height))
                {
                    return 1.0;
                }
            }
            return 0.0;
        }
        if (!upper.spec->shape || !lower.spec->shape)
        {
            throw std::invalid_argument(p_name + "() needs two bodies or two shapes");
        }
        // Props: the bottom face of the upper one on the top face of the
        // lower one, above its footprint.
        Vector3 const a = framePose(upper, {}).position;
        Vector3 const b = framePose(lower, {}).position;
        Vector3 const ha = halfSize(*upper.spec);
        Vector3 const hb = halfSize(*lower.spec);
        bool const stacked = std::abs((a.z - ha.z) - (b.z + hb.z)) < ON_TOLERANCE_M &&
                             std::abs(a.x - b.x) < hb.x && std::abs(a.y - b.y) < hb.y;
        if (p_name == "on")
        {
            return boolean(stacked);
        }
        return boolean(std::abs(a.x - b.x) <= ha.x + hb.x + ON_TOLERANCE_M &&
                       std::abs(a.y - b.y) <= ha.y + hb.y + ON_TOLERANCE_M &&
                       std::abs(a.z - b.z) <= ha.z + hb.z + ON_TOLERANCE_M);
    }
    if (p_name == "collision_count")
    {
        arity(p_name, p_args, 1u, 1u);
        Record const& record = require(p_args[0]);
        int count = record.peak_contacts;
        if (record.spec->type == EntitySpec::Type::Robot && m_prop_penetration)
        {
            ++count;
        }
        return static_cast<double>(count);
    }
    if (p_name == "tool_is_attached")
    {
        arity(p_name, p_args, 2u, 2u);
        EntitySpec const* tool = m_scenario.world.find(p_args[0]);
        if (tool == nullptr || tool->type != EntitySpec::Type::Tool)
        {
            throw std::invalid_argument("'" + p_args[0] + "' is not a tool");
        }
        EntitySpec const* mounted =
            tool->attach ? m_scenario.world.mountedTool(tool->attach->parent) : nullptr;
        bool const attached = mounted == tool &&
                              p_args[1] == mounted->attach->parent + "." + mounted->attach->parent_frame;
        return boolean(attached);
    }
    if (p_name == "joint_limits_respected")
    {
        arity(p_name, p_args, 1u, 1u);
        EntitySpec const& spec = *require(p_args[0]).spec;
        if (spec.type != EntitySpec::Type::Robot)
        {
            throw std::invalid_argument("'" + p_args[0] + "' is not a robot");
        }
        JointSet const& joints = ownerOf(spec).joints();
        for (JointId id = 0; id < joints.size(); ++id)
        {
            double lower = 0.0;
            double upper = 0.0;
            if (joints.isPrismatic(id))
            {
                auto const limits = joints.limits(joints.prismatic(id));
                lower = limits.lower.value();
                upper = limits.upper.value();
            }
            else
            {
                auto const limits = joints.limits(joints.revolute(id));
                lower = limits.lower.value();
                upper = limits.upper.value();
            }
            double const at = joints.position(id);
            if (at < lower - JOINT_LIMIT_TOLERANCE || at > upper + JOINT_LIMIT_TOLERANCE)
            {
                return 0.0;
            }
        }
        return 1.0;
    }
    if (p_name == "holding")
    {
        arity(p_name, p_args, 1u, 2u);
        VacuumGripper const* gripper = nullptr;
        for (std::unique_ptr<RobotSession> const& robot : m_robots)
        {
            gripper = gripper != nullptr ? gripper
                                         : robot->actuators().find<VacuumGripper>(p_args[0]);
        }
        if (gripper == nullptr)
        {
            throw std::invalid_argument("'" + p_args[0] + "' is not a vacuum gripper");
        }
        if (!gripper->holding())
        {
            return 0.0;
        }
        if (p_args.size() == 1u)
        {
            return 1.0;
        }
        compages::world::Entity const held = m_world.entity(gripper->held());
        return boolean(held.name() == p_args[1]);
    }
    if (p_name == "skill_started" || p_name == "skill_succeeded" || p_name == "skill_failed")
    {
        arity(p_name, p_args, 1u, 1u);
        SkillId const id = m_scheduler->find(p_args[0]);
        if (id == NO_SKILL)
        {
            throw std::invalid_argument("unknown skill '" + p_args[0] + "'");
        }
        for (SkillRun const& run : m_scheduler->trace())
        {
            if (run.skill != id)
            {
                continue;
            }
            bool const hit = p_name == "skill_started"
                                 ? run.state != SkillState::Waiting
                                 : run.state == (p_name == "skill_succeeded" ? SkillState::Succeeded
                                                                             : SkillState::Failed);
            if (hit)
            {
                return 1.0;
            }
        }
        return 0.0;
    }
    return std::nullopt;
}

} // namespace robotik
