// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file RoboticCellTruth.cpp
/// Ground-truth queries of @ref RoboticCell: what the assertions
/// measure. World frame, metres and radians.

#include "RobotIK/Common/SExpression.hpp"
#include "RobotIK/Task/RoboticCell.hpp"
#include "ShapeOf.hpp"

#include "RobotIK/Robot/Kinematics.hpp"
#include "RobotIK/Robot/Components/Objects.hpp"
#include "RobotIK/Robot/Actuators.hpp"
#include "RobotIK/Task/Grasp/ContainerBounds.hpp"
#include "RobotIK/Task/Story/Story.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

/// Vertical gap tolerated between two props said to rest on each other (m).
#define ON_TOLERANCE_M 0.005
/// Joint limit overshoot tolerated by joint_limits_respected (rad or m).
#define JOINT_LIMIT_TOLERANCE 1e-3

namespace robotik
{

/// "easel.canvas_mount" -> { "easel", "canvas_mount" }.
static std::pair<std::string_view, std::string_view>
splitReference(std::string_view p_reference)
{
    std::size_t const dot = p_reference.find('.');
    if (dot == std::string_view::npos)
    {
        return { p_reference, {} };
    }
    return { p_reference.substr(0, dot), p_reference.substr(dot + 1U) };
}

/// Angle in radians between two orientations.
static double angleBetween(Quaternion const& p_a, Quaternion const& p_b)
{
    double const dot =
        std::abs(p_a.w * p_b.w + p_a.x * p_b.x + p_a.y * p_b.y + p_a.z * p_b.z);
    return 2.0 * std::acos(std::min(1.0, dot));
}

/// Half the extents of an entity shape, or zero when it has none.
static Vector3 halfSize(EntitySpec const& p_spec)
{
    return p_spec.shape ? Vector3(0.5 * p_spec.shape->size.x,
                                  0.5 * p_spec.shape->size.y,
                                  0.5 * p_spec.shape->size.z)
                        : Vector3::ZERO;
}

RoboticCell::Record const* RoboticCell::record(std::string_view p_id) const
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

RoboticCell::Record const& RoboticCell::require(std::string_view p_id) const
{
    if (Record const* found = record(p_id))
    {
        return *found;
    }
    if (EntitySpec const* spec = findEntity(m_description.world, p_id);
        spec != nullptr && spec->type == EntitySpec::Type::Tool)
    {
        throw std::invalid_argument("tool '" + std::string(p_id) +
                                    "' is not mounted");
    }
    std::vector<std::string_view> known;
    for (EntitySpec const& entity : m_description.world.entities)
    {
        known.push_back(entity.id);
    }
    std::string const closest = closestWord(p_id, known);
    throw std::invalid_argument("unknown entity '" + std::string(p_id) + "'" +
                                (closest.empty()
                                     ? std::string{}
                                     : " (did you mean '" + closest + "'?)"));
}

compages::world::Entity RoboticCell::decorate(ecs::Shape const& p_object)
{
    compages::world::Entity entity = m_world.entity(p_object.name);
    entity.parent(m_frame).set(p_object).set(ecs::Decoration{});
    if (m_view != nullptr)
    {
        m_view->object(entity, p_object);
    }
    return entity;
}

compages::world::Entity RoboticCell::entity(std::string_view p_id)
{
    Record const* found = record(p_id);
    return found != nullptr ? found->entity : compages::world::Entity{};
}

Pose RoboticCell::framePose(Record const& p_record,
                            std::string_view p_frame) const
{
    EntitySpec const& spec = *p_record.spec;
    std::string const frame(p_frame);
    switch (spec.type)
    {
        case EntitySpec::Type::Robot:
            return ownerOf(spec).requireTrueWorldPose(frame);
        case EntitySpec::Type::Tool:
            // A mounted tool is part of the robot chain; its tool_mount lands
            // on the flange (Pinocchio merges the two).
            return ownerOf(spec).requireTrueWorldPose(
                frame.empty() ? p_record.spec->attach->parent_frame : frame);
        case EntitySpec::Type::Prop:
        case EntitySpec::Type::Actor:
        {
            // Where it is now: a carried or pushed prop turns too, the story
            // moves an actor.
            auto const at = p_record.entity.position();
            auto const turn = p_record.entity.rotation();
            Pose const base{ Vector3(static_cast<double>(at.x),
                                     static_cast<double>(at.y),
                                     static_cast<double>(at.z)),
                             Quaternion(static_cast<double>(turn.w),
                                        static_cast<double>(turn.x),
                                        static_cast<double>(turn.y),
                                        static_cast<double>(turn.z)) };
            if (frame.empty())
            {
                return base;
            }
            auto const point = spec.actor.frames.find(frame);
            if (spec.type == EntitySpec::Type::Prop ||
                point == spec.actor.frames.end())
            {
                throw std::invalid_argument("'" + spec.id + "' has no frame '" +
                                            frame + "'");
            }
            return base * Pose{ point->second, Quaternion{} };
        }
        case EntitySpec::Type::Body:
            break;
    }
    PhysicsWorld const* physics = m_robot->getPhysics();
    std::optional<Pose> const root =
        physics != nullptr ? physics->bodyPose(p_record.body) : std::nullopt;
    Pose const base = root ? *root : p_record.nominal;
    if (frame.empty())
    {
        return base;
    }
    if (p_record.frames == nullptr || !p_record.frames->hasFrame(frame))
    {
        throw std::invalid_argument("'" + spec.id + "' has no frame '" + frame +
                                    "'");
    }
    return base * p_record.frames->requireFramePose(frame);
}

Pose RoboticCell::requirePose(std::string_view p_reference) const
{
    auto const [id, frame] = splitReference(p_reference);
    return framePose(require(id), frame);
}

Pose RoboticCell::initialPose(std::string_view p_id) const
{
    return require(p_id).initial;
}

Pose RoboticCell::requireDescribedPose(std::string_view p_reference) const
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
                throw std::invalid_argument("prop '" + record.spec->id +
                                            "' has no frame '" +
                                            std::string(frame) + "'");
            }
            return record.nominal;
        case EntitySpec::Type::Actor:
        {
            if (frame.empty())
            {
                return record.nominal;
            }
            auto const point = record.spec->actor.frames.find(frame);
            if (point == record.spec->actor.frames.end())
            {
                throw std::invalid_argument("'" + record.spec->id +
                                            "' has no frame '" +
                                            std::string(frame) + "'");
            }
            return record.nominal * Pose{ point->second, Quaternion{} };
        }
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
        throw std::invalid_argument("'" + record.spec->id + "' has no frame '" +
                                    link + "'");
    }
    return record.nominal * record.frames->requireFramePose(link);
}

std::vector<Pose> RoboticCell::requireWaypoints(std::string_view p_name) const
{
    auto const found = m_description.trajectories.find(p_name);
    if (found == m_description.trajectories.end())
    {
        throw std::invalid_argument("the scenario has no trajectory '" +
                                    std::string(p_name) + "'");
    }
    Trajectory const& trajectory = found->second;
    if (trajectory.type != Trajectory::Type::Cartesian)
    {
        throw std::invalid_argument("trajectory '" + std::string(p_name) +
                                    "' is not cartesian");
    }
    Pose const frame = trajectory.frame.empty()
                           ? Pose{ Vector3::ZERO, Quaternion{} }
                           : requireDescribedPose(trajectory.frame);
    Pose const to_base = m_robot->robot().getBase().pose.inverse() * frame;
    std::vector<Pose> result;
    result.reserve(trajectory.poses.size());
    for (Pose const& point : trajectory.poses)
    {
        result.push_back(to_base * point);
    }
    return result;
}

/// Robot an entity belongs to: its own id for a robot, the robot it
/// is mounted on for a tool, empty otherwise.
static std::string_view assemblyOf(EntitySpec const& p_spec)
{
    if (p_spec.type == EntitySpec::Type::Robot)
    {
        return p_spec.id;
    }
    if (p_spec.type == EntitySpec::Type::Tool && p_spec.attach)
    {
        return p_spec.attach->parent;
    }
    return {};
}

int RoboticCell::contactsOf(Record const& p_record,
                            std::vector<Contact> const& p_contacts) const
{
    // A robot and its mounted tool are one assembly: a tool hitting the world
    // is a robot collision too, the robot touching its own tool is not.
    // Another robot, or its tool, is the world.
    std::string_view const assembly = assemblyOf(*p_record.spec);
    auto inAssembly = [&](std::string const& p_body)
    {
        return !assembly.empty() &&
               std::ranges::any_of(m_records,
                                   [&](Record const& p_peer)
                                   {
                                       return assemblyOf(*p_peer.spec) ==
                                                  assembly &&
                                              p_peer.links.contains(p_body);
                                   });
    };
    auto mine = [&](std::string const& p_body)
    {
        return p_record.links.contains(p_body) ||
               (p_record.spec->type == EntitySpec::Type::Robot &&
                inAssembly(p_body));
    };
    // A tool on the entities it works on (a drill on its workpiece).
    auto works =
        [&](std::string const& p_tool_body, std::string const& p_other_body)
    {
        auto const tool = std::ranges::find_if(
            m_records,
            [&](Record const& p_peer)
            {
                return p_peer.spec->type == EntitySpec::Type::Tool &&
                       p_peer.links.contains(p_tool_body);
            });
        return tool != m_records.end() &&
               std::ranges::any_of(tool->spec->touches,
                                   [&](std::string const& p_id)
                                   {
                                       Record const* workpiece = record(p_id);
                                       return workpiece != nullptr &&
                                              workpiece->links.contains(
                                                  p_other_body);
                                   });
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
        if (inAssembly(first ? contact.second : contact.first))
        {
            continue;
        }
        if (works(contact.first, contact.second) ||
            works(contact.second, contact.first))
        {
            continue;
        }
        ++count;
    }
    return count;
}

Metrics RoboticCell::metrics() const
{
    Metrics metrics;
    metrics.set("time", time().value());
    metrics.set("controller.success",
                m_status == Status::SUCCESS ? 1.0 : 0.0);
    if (m_executor)
    {
        metrics.set("planner.plans",
                    static_cast<double>(m_executor->history().size()));
        metrics.set("planner.replans",
                    static_cast<double>(m_executor->replans()));
        metrics.set("planner.steps",
                    static_cast<double>(m_executor->plan().size()));
        metrics.set("planner.done", static_cast<double>(m_executor->done()));
    }
    if (m_logic != nullptr)
    {
        m_logic->measure(*this, metrics);
    }
    return metrics;
}

ConditionVocabulary RoboticCell::vocabulary() const
{
    ConditionVocabulary vocabulary;
    Metrics const table = metrics();
    vocabulary.metrics.assign(table.names().begin(), table.names().end());
    vocabulary.queries = m_queries;
    return vocabulary;
}

Result<Condition> RoboticCell::compile(std::string_view p_text) const
{
    return compileCondition(p_text, vocabulary());
}

Evaluation RoboticCell::evaluate(std::string_view p_condition) const
{
    Result<Condition> condition = compile(p_condition);
    if (!condition)
    {
        Evaluation failed;
        failed.detail = condition.error();
        return failed;
    }
    return robotik::evaluate(condition.value(), metrics());
}

std::optional<double> RoboticCell::requireQuery(std::string_view p_term) const
{
    Result<Condition> condition = compile(p_term);
    if (!condition)
    {
        throw std::invalid_argument(condition.error());
    }
    Evaluation const result = robotik::evaluate(condition.value(), metrics());
    if (!result.known)
    {
        return std::nullopt;
    }
    return result.value;
}

std::optional<double>
RoboticCell::requireQuery(std::string_view p_name,
                          std::vector<std::string> const& p_args) const
{
    for (ConditionQuery const& query : m_queries)
    {
        if (query.name != p_name)
        {
            continue;
        }
        if (p_args.size() < query.min_args || p_args.size() > query.max_args)
        {
            throw std::invalid_argument(
                "(" + query.name + " ...) takes " +
                std::to_string(query.min_args) +
                (query.min_args == query.max_args
                     ? ""
                     : " to " + std::to_string(query.max_args)) +
                " argument(s)");
        }
        return query.answer(p_args);
    }
    throw std::invalid_argument("unknown query '" + std::string(p_name) + "'");
}

void RoboticCell::addQuery(ConditionQuery p_query)
{
    for (ConditionQuery const& query : m_queries)
    {
        if (query.name == p_query.name)
        {
            throw std::invalid_argument("query '" + p_query.name +
                                        "' is declared twice");
        }
    }
    m_queries.push_back(std::move(p_query));
}

std::vector<ConditionQuery> RoboticCell::builtinQueries() const
{
    using Arguments = std::vector<std::string>;
    auto boolean = [](bool p_value) { return p_value ? 1.0 : 0.0; };
    std::vector<ConditionQuery> queries;

    auto error = [this](Arguments const& p_args,
                        bool p_angle) -> std::optional<double>
    {
        Record const& record = require(p_args[0]);
        Pose const now = framePose(record, {});
        if (!p_angle)
        {
            return norm(now.position - record.initial.position);
        }
        return angleBetween(now.rotation, record.initial.rotation);
    };
    queries.push_back({ "pose_error", 1U, 1U,
                        [error](Arguments const& p_args)
                        { return error(p_args, false); } });
    queries.push_back({ "orientation_error", 1U, 1U,
                        [error](Arguments const& p_args)
                        { return error(p_args, true); } });

    queries.push_back({ "distance",
                        2U,
                        2U,
                        [this](Arguments const& p_args) -> std::optional<double>
                        {
                            return norm(requirePose(p_args[0]).position -
                                        requirePose(p_args[1]).position);
                        } });

    queries.push_back(
        { "inside", 2U, 2U,
          [this, boolean](Arguments const& p_args) -> std::optional<double>
          {
              Record const& object = require(p_args[0]);
              Record const& container = require(p_args[1]);
              if (!object.spec->shape || !container.spec->shape)
              {
                  throw std::invalid_argument(
                      "(inside a b) compares two primitive shapes");
              }
              Vector3 const at = framePose(object, {}).position;
              Vector3 const center = framePose(container, {}).position;
              if (container.spec->shape->type == ShapeSpec::Type::Container)
              {
                  return boolean(scene::restsInside(shapeOf(*container.spec),
                                                    center,
                                                    at,
                                                    halfSize(*object.spec)));
              }
              Vector3 const half = halfSize(*container.spec);
              return boolean(std::abs(at.x - center.x) < half.x &&
                             std::abs(at.y - center.y) < half.y &&
                             std::abs(at.z - center.z) < half.z);
          } });

    auto contact = [this, boolean](Arguments const& p_args,
                                   bool p_on) -> std::optional<double>
    {
        std::string const name = p_on ? "on" : "touching";
        Record const& upper = require(p_args[0]);
        Record const& lower = require(p_args[1]);
        PhysicsWorld const* physics = m_robot->getPhysics();
        if (!upper.links.empty() && !lower.links.empty() && physics != nullptr)
        {
            // Bodies of the physics backend: a contact between them, below the
            // upper one for "on".
            double const height = framePose(upper, {}).position.z;
            for (Contact const& c : physics->contactList())
            {
                bool const between = (upper.links.contains(c.first) &&
                                      lower.links.contains(c.second)) ||
                                     (upper.links.contains(c.second) &&
                                      lower.links.contains(c.first));
                if (between && (!p_on || c.position.z < height))
                {
                    return 1.0;
                }
            }
            return 0.0;
        }
        if (!upper.spec->shape || !lower.spec->shape)
        {
            throw std::invalid_argument("(" + name +
                                        " a b) needs two bodies or two shapes");
        }
        // Props: the bottom face of the upper one on the top face of the
        // lower one, above its footprint.
        Vector3 const a = framePose(upper, {}).position;
        Vector3 const b = framePose(lower, {}).position;
        Vector3 const ha = halfSize(*upper.spec);
        Vector3 const hb = halfSize(*lower.spec);
        bool const stacked =
            std::abs((a.z - ha.z) - (b.z + hb.z)) < ON_TOLERANCE_M &&
            std::abs(a.x - b.x) < hb.x && std::abs(a.y - b.y) < hb.y;
        if (p_on)
        {
            return boolean(stacked);
        }
        return boolean(std::abs(a.x - b.x) <= ha.x + hb.x + ON_TOLERANCE_M &&
                       std::abs(a.y - b.y) <= ha.y + hb.y + ON_TOLERANCE_M &&
                       std::abs(a.z - b.z) <= ha.z + hb.z + ON_TOLERANCE_M);
    };
    queries.push_back({ "touching", 2U, 2U,
                        [contact](Arguments const& p_args)
                        { return contact(p_args, false); } });
    queries.push_back({ "on", 2U, 2U,
                        [contact](Arguments const& p_args)
                        { return contact(p_args, true); } });

    queries.push_back(
        { "collision_count", 1U, 1U,
          [this](Arguments const& p_args) -> std::optional<double>
          {
              Record const& record = require(p_args[0]);
              int count = record.peak_contacts;
              if (record.spec->type == EntitySpec::Type::Robot &&
                  m_prop_penetration.contains(record.spec->id))
              {
                  ++count;
              }
              return static_cast<double>(count);
          } });

    queries.push_back(
        { "tool_is_attached", 2U, 2U,
          [this, boolean](Arguments const& p_args) -> std::optional<double>
          {
              EntitySpec const* tool =
                  findEntity(m_description.world, p_args[0]);
              if (tool == nullptr || tool->type != EntitySpec::Type::Tool)
              {
                  throw std::invalid_argument("'" + p_args[0] +
                                              "' is not a tool");
              }
              EntitySpec const* mounted =
                  tool->attach
                      ? mountedTool(m_description.world, tool->attach->parent)
                      : nullptr;
              bool const attached =
                  mounted == tool &&
                  p_args[1] == mounted->attach->parent + "." +
                                   mounted->attach->parent_frame;
              return boolean(attached);
          } });

    queries.push_back(
        { "joint_limits_respected", 1U, 1U,
          [this](Arguments const& p_args) -> std::optional<double>
          {
              EntitySpec const& spec = *require(p_args[0]).spec;
              if (spec.type != EntitySpec::Type::Robot)
              {
                  throw std::invalid_argument("'" + p_args[0] +
                                              "' is not a robot");
              }
              JointSet const& joints = ownerOf(spec).robot().getJoints();
              for (JointId id = 0; id < joints.size(); ++id)
              {
                  double lower = 0.0;
                  double upper = 0.0;
                  if (joints.isPrismatic(id))
                  {
                      auto const limits =
                          joints.limits(joints.requirePrismatic(id));
                      lower = limits.lower.value();
                      upper = limits.upper.value();
                  }
                  else
                  {
                      auto const limits =
                          joints.limits(joints.requireRevolute(id));
                      lower = limits.lower.value();
                      upper = limits.upper.value();
                  }
                  double const at = joints.position(id);
                  if (at < lower - JOINT_LIMIT_TOLERANCE ||
                      at > upper + JOINT_LIMIT_TOLERANCE)
                  {
                      return 0.0;
                  }
              }
              return 1.0;
          } });

    queries.push_back(
        { "holding", 1U, 2U,
          [this, boolean](Arguments const& p_args) -> std::optional<double>
          {
              VacuumGripper const* gripper = nullptr;
              for (std::unique_ptr<RobotRuntime> const& robot : m_robots)
              {
                  gripper =
                      gripper != nullptr
                          ? gripper
                          : robot->robot().getActuators().find<VacuumGripper>(
                                p_args[0]);
              }
              if (gripper == nullptr)
              {
                  throw std::invalid_argument("'" + p_args[0] +
                                              "' is not a vacuum gripper");
              }
              if (!gripper->holding())
              {
                  return 0.0;
              }
              if (p_args.size() == 1U)
              {
                  return 1.0;
              }
              compages::world::Entity const held =
                  m_world.entity(m_held.of(*gripper));
              return boolean(held.name() == p_args[1]);
          } });

    auto skill = [this](Arguments const& p_args,
                        int p_kind) -> std::optional<double>
    {
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
            bool const hit = p_kind == 0
                                 ? run.state != SkillState::Waiting
                                 : run.state == (p_kind == 1
                                                     ? SkillState::Succeeded
                                                     : SkillState::Failed);
            if (hit)
            {
                return 1.0;
            }
        }
        return 0.0;
    };
    queries.push_back({ "skill_started", 1U, 1U,
                        [skill](Arguments const& p_args)
                        { return skill(p_args, 0); } });
    queries.push_back({ "skill_succeeded", 1U, 1U,
                        [skill](Arguments const& p_args)
                        { return skill(p_args, 1); } });
    queries.push_back({ "skill_failed", 1U, 1U,
                        [skill](Arguments const& p_args)
                        { return skill(p_args, 2); } });

    // --- What the robot believes, to compare with the truth ---------------
    // (believed (in red_cube box)): the observer of the planner on the
    // beliefs; no answer when it cannot tell.
    queries.push_back(
        { "believed",
          1U,
          std::numeric_limits<std::size_t>::max(),
          [this](Arguments const& p_args) -> std::optional<double>
          {
              Observer const* observer = m_planning.observer(p_args[0]);
              if (observer == nullptr)
              {
                  throw std::invalid_argument(
                      "no observer for the predicate '" + p_args[0] +
                      "' (the scenario logic gives them to the planner)");
              }
              Args const args(p_args.begin() + 1, p_args.end());
              switch ((*observer)(m_contexts.front(), args))
              {
                  case Truth::True:
                      return 1.0;
                  case Truth::False:
                      return 0.0;
                  default:
                      return std::nullopt;
              }
          },
          true });
    // Distance between where the robot believes an object is and where it
    // is. No answer when the object was never seen, its belief is older
    // than perception.max_age, or a gripper carries it (nobody tracks it).
    queries.push_back(
        { "belief_error",
          1U,
          1U,
          [this](Arguments const& p_args) -> std::optional<double>
          {
              Record const& record = require(p_args[0]);
              WorldObject const* belief = m_world_model.find(p_args[0]);
              if (belief == nullptr || !observed(*belief) ||
                  m_robot->robot().getTime() - belief->seen >
                      m_description.perception.max_age)
              {
                  return std::nullopt;
              }
              for (std::unique_ptr<RobotRuntime> const& robot : m_robots)
              {
                  ActuatorSet const& actuators = robot->robot().getActuators();
                  for (std::size_t i = 0; i < actuators.size(); ++i)
                  {
                      auto const* cup =
                          dynamic_cast<VacuumGripper const*>(&actuators[i]);
                      if (cup != nullptr && record.entity &&
                          m_held.of(*cup) == record.entity.id())
                      {
                          return std::nullopt;
                      }
                  }
              }
              return norm(beliefFrame() * belief->position -
                          framePose(record, {}).position);
          } });
    // Distance between the estimated and the true base of a robot (see
    // LocalizationFidelity).
    queries.push_back({ "base_error",
                        1U,
                        1U,
                        [this](Arguments const& p_args) -> std::optional<double>
                        {
                            RobotRuntime const* found = runtime(p_args[0]);
                            if (found == nullptr)
                            {
                                throw std::invalid_argument("'" + p_args[0] +
                                                            "' is not a robot");
                            }
                            return norm(found->robot().getBase().pose.position -
                                        found->getTrueBase().pose.position);
                        } });
    storyQueries(queries);
    return queries;
}

void RoboticCell::storyQueries(std::vector<ConditionQuery>& p_queries) const
{
    using Arguments = std::vector<std::string>;
    auto boolean = [](bool p_value) { return p_value ? 1.0 : 0.0; };
    StorySpec const* story = &m_description.story;
    // The arguments are checked on the description: the conditions of the
    // story itself are compiled before the story exists, and read 0 then.
    auto event = [story](std::string const& p_id)
    {
        if (std::ranges::none_of(story->events,
                                 [&](StoryEvent const& p_event)
                                 { return p_event.id == p_id; }))
        {
            throw std::invalid_argument("the story has no event '" + p_id +
                                        "'");
        }
    };
    auto actor = [this](std::string const& p_id)
    {
        EntitySpec const* spec = findEntity(m_description.world, p_id);
        if (spec == nullptr || spec->type != EntitySpec::Type::Actor)
        {
            throw std::invalid_argument("'" + p_id +
                                        "' is no actor of the world");
        }
    };
    auto eventQuery = [this, event](char const* p_name, auto p_answer)
    {
        return ConditionQuery{
            p_name,
            1U,
            1U,
            [this, event, p_answer](
                Arguments const& p_args) -> std::optional<double>
            {
                event(p_args[0]);
                return m_story ? p_answer(*m_story, p_args[0]) : 0.0;
            }
        };
    };
    p_queries.push_back(
        eventQuery("event_started",
                   [boolean](Story const& p_story, std::string const& p_id)
                   { return boolean(p_story.requireStarts(p_id) > 0U); }));
    p_queries.push_back(
        eventQuery("event_running",
                   [boolean](Story const& p_story, std::string const& p_id)
                   {
                       EventPhase const phase = p_story.requirePhase(p_id);
                       return boolean(phase == EventPhase::Running ||
                                      phase == EventPhase::Delaying);
                   }));
    p_queries.push_back(
        eventQuery("event_done",
                   [boolean](Story const& p_story, std::string const& p_id)
                   { return boolean(p_story.requireCompletions(p_id) > 0U); }));
    p_queries.push_back(
        eventQuery("event_failed",
                   [boolean](Story const& p_story, std::string const& p_id)
                   { return boolean(p_story.requireFailures(p_id) > 0U); }));
    p_queries.push_back(eventQuery(
        "event_count",
        [](Story const& p_story, std::string const& p_id)
        { return static_cast<double>(p_story.requireCompletions(p_id)); }));
    p_queries.push_back(
        { "story_var",
          1U,
          1U,
          [this, story](Arguments const& p_args) -> std::optional<double>
          {
              auto const found = story->variables.find(p_args[0]);
              if (found == story->variables.end())
              {
                  throw std::invalid_argument("the story has no variable '" +
                                              p_args[0] + "'");
              }
              return m_story ? m_story->requireVariable(p_args[0])
                             : found->second;
          } });
    p_queries.push_back(
        { "said",
          2U,
          2U,
          [this, boolean](Arguments const& p_args) -> std::optional<double>
          {
              static_cast<void>(require(p_args[0]));
              return boolean(m_story && m_story->said(p_args[0], p_args[1]));
          } });
    auto actorQuery = [this, actor](char const* p_name, auto p_answer)
    {
        return ConditionQuery{
            p_name,
            1U,
            1U,
            [this, actor, p_answer](
                Arguments const& p_args) -> std::optional<double>
            {
                actor(p_args[0]);
                return m_story ? p_answer(*m_story, p_args[0]) : 0.0;
            }
        };
    };
    p_queries.push_back(
        actorQuery("speaking",
                   [boolean](Story const& p_story, std::string const& p_id)
                   { return boolean(p_story.requireSpeaking(p_id)); }));
    p_queries.push_back(
        actorQuery("shown",
                   [boolean](Story const& p_story, std::string const& p_id)
                   { return boolean(p_story.requireShown(p_id)); }));
    p_queries.push_back(
        actorQuery("actor_speed",
                   [](Story const& p_story, std::string const& p_id)
                   { return p_story.requireSpeed(p_id); }));
    p_queries.push_back(
        { "expression",
          2U,
          2U,
          [this, actor](Arguments const& p_args) -> std::optional<double>
          {
              actor(p_args[0]);
              return m_story ? m_story->expression(p_args[0], p_args[1]) : 0.0;
          } });
    // Where a target of the story is: a place, an entity, entity.frame.
    auto point = [this, story](std::string const& p_target)
    {
        if (auto const place = story->places.find(p_target);
            place != story->places.end())
        {
            return place->second;
        }
        return requirePose(p_target).position;
    };
    p_queries.push_back(
        { "at",
          2U,
          3U,
          [this, point, boolean](
              Arguments const& p_args) -> std::optional<double>
          {
              Vector3 const a = requirePose(p_args[0]).position;
              Vector3 const b = point(p_args[1]);
              double tolerance = 0.5;
              if (p_args.size() == 3U)
              {
                  try
                  {
                      tolerance = std::stod(p_args[2]);
                  }
                  catch (std::exception const&)
                  {
                      throw std::invalid_argument("(at a b tolerance): '" +
                                                  p_args[2] + "' is no number");
                  }
              }
              return boolean(std::hypot(b.x - a.x, b.y - a.y) <= tolerance);
          } });
    p_queries.push_back(
        { "heading_error",
          2U,
          2U,
          [this, point](Arguments const& p_args) -> std::optional<double>
          {
              Pose const a = requirePose(p_args[0]);
              Vector3 const b = point(p_args[1]);
              double const toward =
                  std::atan2(b.y - a.position.y, b.x - a.position.x);
              return std::abs(std::remainder(toward - yaw(a.rotation).value(),
                                             2.0 * 3.14159265358979323846));
          } });
}

} // namespace robotik
