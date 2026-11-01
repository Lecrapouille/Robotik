// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "ConveyorMission.hpp"

#include "ArmSkills.hpp"

#include "Robotik/Task/Cell.hpp"
#include "Robotik/Task/CellDevices.hpp"
#include "Robotik/Task/Metrics.hpp"
#include "Robotik/Task/Perception/WorldModel.hpp"
#include "Robotik/Task/Planning/PlanningDomain.hpp"
#include "Robotik/Task/RobotContext.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace dual_arm
{

/// Height of the cup over a part before and after the grasp (m).
static constexpr double HOVER = 0.08;
/// Height of the cup over a part to grasp it: the cup takes it within 2 cm,
/// and a lower cup would push the part through the floor of its tray.
static constexpr double CONTACT = 0.012;
/// Height of the cup over the walls of a bin when it drops a part (m).
static constexpr double OVER_BIN = 0.12;
/// Height of a part (m): an arm waits over its pick zone at the height it
/// takes the next part from, HOVER above its top.
static constexpr double PART_HEIGHT = 0.04;
/// Room between the bottom of a part and a belt when an arm puts it there
/// (m), and between the part and the upstream end of the belt.
static constexpr double OVER_BELT = 0.01;
static constexpr double BELT_MARGIN = 0.04;
static constexpr int PRIORITY = 100;

/// A skill named @p_name on robot @p_robot holding @p_resources.
static robotik::SkillDescription
describe(std::string p_name,
         std::string p_robot,
         std::vector<robotik::ResourceRequirement> p_resources)
{
    robotik::SkillDescription description;
    description.name = std::move(p_name);
    description.robot = std::move(p_robot);
    description.resources = std::move(p_resources);
    description.priority = PRIORITY;
    return description;
}

/// True when @p_point lies over the footprint of the box body @p_body.
static bool above(robotik::Vector3 const& p_point,
                  robotik::EntitySpec const& p_body)
{
    robotik::Vector3 const local = p_body.pose.inverse() * p_point;
    robotik::Vector3 const& size = p_body.shape->size;
    return std::abs(local.x) <= 0.5 * size.x &&
           std::abs(local.y) <= 0.5 * size.y && local.z >= 0.0;
}

void ConveyorMission::setup(robotik::Cell& p_cell,
                            robotik::SceneView* /*p_view*/)
{
    for (robotik::EntitySpec const& entity :
         p_cell.description().world.entities)
    {
        if (entity.kind == "part")
        {
            m_parts.push_back(entity.id);
        }
        else if (entity.kind == "feeder")
        {
            m_feeders.push_back(entity.id);
        }
        else if (entity.kind == "zone")
        {
            m_zones.push_back(entity.id);
        }
        else if (entity.kind == "bin")
        {
            m_bins.push_back(entity.id);
        }
        // A belt is fixed furniture: its entry comes from the layout.
        for (robotik::ActuatorSpec const& actuator : entity.actuators)
        {
            if (actuator.type != robotik::ActuatorSpec::Type::Conveyor ||
                !entity.shape)
            {
                continue;
            }
            robotik::Vector3 const& size = entity.shape->size;
            double const length =
                std::hypot(actuator.direction.x, actuator.direction.y);
            double const dx = actuator.direction.x / length;
            double const dy = actuator.direction.y / length;
            double const half =
                0.5 * (std::abs(dx) * size.x + std::abs(dy) * size.y);
            double const back = half - BELT_MARGIN;
            m_belt_entries.emplace_back(
                entity.id,
                entity.pose *
                    robotik::Vector3(-dx * back, -dy * back, 0.5 * size.z));
        }
    }

    // The pusher is a resource of the cell, next to the arms, the cups and
    // the belts of the lanes.
    robotik::ResourceManager& resources = p_cell.robot().resources();
    robotik::ResourceRequirement const pusher = {
        resources.add(Pusher::RESOURCE), robotik::Access::Exclusive
    };
    for (std::string const& zone : m_zones)
    {
        // The lane of a zone: the belt under it, in the layout.
        robotik::EntitySpec const* place =
            robotik::findEntity(p_cell.description().world, zone);
        robotik::ConveyorBelt* lane = nullptr;
        for (robotik::EntitySpec const& entity :
             p_cell.description().world.entities)
        {
            for (robotik::ActuatorSpec const& actuator : entity.actuators)
            {
                if (actuator.type == robotik::ActuatorSpec::Type::Conveyor &&
                    entity.shape && place != nullptr &&
                    above(place->pose.position, entity))
                {
                    lane = p_cell.devices().find<robotik::ConveyorBelt>(
                        actuator.name);
                }
            }
        }
        if (lane == nullptr)
        {
            throw std::runtime_error("zone '" + zone +
                                     "' is on no conveyor belt");
        }
        std::vector<robotik::ResourceRequirement> const needs = {
            pusher, resources.require(lane->name())
        };
        for (std::string const& part : m_parts)
        {
            for (std::string const& feeder : m_feeders)
            {
                p_cell.skills().add<ConveySkill>(describe("Convey(" + part +
                                                              ", " + feeder +
                                                              ", " + zone + ")",
                                                          {},
                                                          needs),
                                                 m_pusher,
                                                 *lane,
                                                 part,
                                                 feeder,
                                                 zone);
            }
        }
    }
    for (std::unique_ptr<robotik::RobotSession> const& robot :
         p_cell.robots())
    {
        addArmSkills(p_cell, *robot);
    }
    addPlanning(p_cell);
}

void ConveyorMission::addArmSkills(robotik::Cell& p_cell,
                                   robotik::Robot const& p_robot)
{
    robotik::ResourceManager const& resources = p_robot.resources();
    robotik::ActuatorSet const& actuators = p_robot.actuators();
    std::vector<robotik::ResourceRequirement> arm;
    if (auto const* group = actuators.first<robotik::JointGroup>())
    {
        arm.push_back(resources.require(group->name()));
    }
    std::vector<robotik::ResourceRequirement> cup;
    if (auto const* vacuum = actuators.first<robotik::VacuumGripper>())
    {
        cup.push_back(resources.require(vacuum->name()));
    }

    // "left.Hover(p1)": one family per robot, named after its id.
    std::string const& id = p_robot.id();
    std::string const scope = id + ".";
    robotik::SkillScheduler& skills = p_cell.skills();
    skills.add<ReleaseSkill>(describe(scope + "Release", id, cup));
    for (std::string const& part : m_parts)
    {
        skills.add<HoverSkill>(describe(scope + "Hover(" + part + ")", id, arm),
                               part,
                               robotik::Length(HOVER));
        skills.add<HoverSkill>(describe(scope + "Reach(" + part + ")", id, arm),
                               part,
                               robotik::Length(CONTACT));
        robotik::SkillDescription grasp =
            describe(scope + "Grasp(" + part + ")", id, cup);
        grasp.wait = false;
        skills.add<GraspSkill>(std::move(grasp), part);
    }
    for (std::string const& bin : m_bins)
    {
        skills.add<HoverSkill>(describe(scope + "Over(" + bin + ")", id, arm),
                               bin,
                               robotik::Length(OVER_BIN));
    }
    // Over the entry of a belt, the part held just above it.
    for (auto const& [belt, entry] : m_belt_entries)
    {
        skills.add<HoverSkill>(describe(scope + "Over(" + belt + ")", id, arm),
                               entry,
                               belt + " entry",
                               robotik::Length(PART_HEIGHT + OVER_BELT));
    }
    // Back over a pick zone, where the next part arrives.
    for (std::string const& zone : m_zones)
    {
        skills.add<HoverSkill>(describe(scope + "Ready(" + zone + ")", id, arm),
                               zone,
                               robotik::Length(PART_HEIGHT + HOVER));
    }
}

/// True when the believed centre of @p_object lies in the footprint
/// of @p_place (shared beliefs: same frame for both).
static bool onFootprint(robotik::WorldObject const& p_object,
                        robotik::WorldObject const& p_place)
{
    return std::abs(p_object.position.x - p_place.position.x) <
               0.5 * p_place.size.x &&
           std::abs(p_object.position.y - p_place.position.y) <
               0.5 * p_place.size.y;
}

/// An arm of the cell holds @p_part.
static bool heldByAnyone(robotik::RobotContext const& p_context,
                         std::string const& p_part)
{
    if (heldObject(p_context.robot) == p_part)
    {
        return true;
    }
    for (robotik::Robot const* robot : p_context.team)
    {
        if (heldObject(*robot) == p_part)
        {
            return true;
        }
    }
    return false;
}

/// at(part, place) on the beliefs: on its footprint, not held. A
/// part not seen yet is where the world file says (the prior).
static robotik::Truth resting(robotik::RobotContext const& p_context,
                              std::string const& p_part,
                              std::string const& p_place)
{
    robotik::WorldObject const* part = p_context.world_model.find(p_part);
    robotik::WorldObject const* place = p_context.world_model.find(p_place);
    if (part == nullptr || place == nullptr)
    {
        return robotik::Truth::Unknown;
    }
    if (heldByAnyone(p_context, p_part))
    {
        return robotik::Truth::False;
    }
    return robotik::truth(onFootprint(*part, *place));
}

void ConveyorMission::addPlanning(robotik::Cell& p_cell)
{
    using robotik::Args;
    using robotik::RobotContext;
    using robotik::Truth;

    robotik::PlanningDomain& domain = p_cell.planning();

    // --- State, on the shared beliefs and what each arm measures -----------
    domain.observe("free",
                   [](RobotContext const& p_context, Args p_args)
                   {
                       robotik::Robot const* arm =
                           teammate(p_context, p_args[0]);
                       return arm == nullptr
                                  ? Truth::Unknown
                                  : robotik::truth(heldObject(*arm).empty());
                   });
    domain.observe(
        "holding",
        [](RobotContext const& p_context, Args p_args)
        {
            robotik::Robot const* arm = teammate(p_context, p_args[0]);
            return arm == nullptr
                       ? Truth::Unknown
                       : robotik::truth(heldObject(*arm) == p_args[1]);
        });
    domain.observe("at",
                   [](RobotContext const& p_context, Args p_args)
                   { return resting(p_context, p_args[0], p_args[1]); });
    domain.observe("zone_free",
                   [this](RobotContext const& p_context, Args p_args)
                   {
                       for (std::string const& part : m_parts)
                       {
                           if (resting(p_context, part, p_args[0]) ==
                               Truth::True)
                           {
                               return Truth::False;
                           }
                       }
                       return Truth::True;
                   });

    // --- Actions, as skills of each robot ----------------------------------
    domain.bind("convey", { "Convey({0}, {1}, {2})" });
    domain.bind("pick",
                { "{0}.Hover({1})",
                  "{0}.Reach({1})",
                  "{0}.Grasp({1})",
                  "{0}.Hover({1})" });
    domain.bind("place", { "{0}.Over({2})", "{0}.Release", "{0}.Ready({3})" });
}

void ConveyorMission::reset(robotik::Cell& /*p_cell*/,
                            robotik::Seed /*p_seed*/)
{
    m_pusher.reset();
    m_overlap = 0.0;
}

void ConveyorMission::step(robotik::Cell& p_cell, robotik::Seconds p_dt)
{
    m_pusher.step(p_cell, p_dt);

    // Both arms busy: a skill of each robot is running.
    robotik::SkillScheduler const& skills = p_cell.skills();
    std::vector<std::string> busy;
    for (robotik::SkillId id = 0; id < skills.size(); ++id)
    {
        std::string const& robot = skills.description(id).robot;
        if (!robot.empty() &&
            skills.state(id) == robotik::SkillState::Running &&
            std::ranges::find(busy, robot) == busy.end())
        {
            busy.push_back(robot);
        }
    }
    if (busy.size() >= 2U && !p_cell.controllerStopped())
    {
        m_overlap += p_dt.value();
    }
}

void ConveyorMission::measure(robotik::Cell const& /*p_cell*/,
                              robotik::Metrics& p_metrics) const
{
    p_metrics.set("dual_arm.overlap", m_overlap);
}

} // namespace dual_arm
