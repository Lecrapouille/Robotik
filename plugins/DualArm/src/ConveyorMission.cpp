// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "ConveyorMission.hpp"

#include "ArmSkills.hpp"

#include "Robotik/Perception/WorldModel.hpp"
#include "Robotik/Planning/PlanningDomain.hpp"
#include "Robotik/Runtime/Metrics.hpp"
#include "Robotik/Runtime/RobotContext.hpp"
#include "Robotik/Runtime/Simulation.hpp"

#include <algorithm>
#include <cmath>

namespace dual_arm
{

/// Height of the cup over a part before and after the grasp (m).
static constexpr double kHover = 0.08;
/// Height of the cup over a part to grasp it: the cup takes it within 2 cm,
/// and a lower cup would push the part through the floor of its tray.
static constexpr double kContact = 0.012;
/// Height of the cup over the walls of a bin when it drops a part (m).
static constexpr double kOverBin = 0.12;
/// Height of a part (m): an arm waits over its pick zone at the height it
/// takes the next part from, kHover above its top.
static constexpr double kPartHeight = 0.04;
static constexpr int kPriority = 100;

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
    description.priority = kPriority;
    return description;
}

void ConveyorMission::setup(robotik::Simulation& p_simulation,
                            robotik::SceneView* /*p_view*/)
{
    for (robotik::EntitySpec const& entity :
         p_simulation.scenario().world.entities)
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
    }

    // The belt is a resource of the cell, next to the arms and the cups.
    robotik::ResourceManager& resources = p_simulation.robot().resources();
    std::vector<robotik::ResourceRequirement> const belt = {
        { resources.add(Conveyor::RESOURCE), robotik::Access::Exclusive }
    };
    for (std::string const& part : m_parts)
    {
        for (std::string const& feeder : m_feeders)
        {
            for (std::string const& zone : m_zones)
            {
                p_simulation.skills().add<ConveySkill>(
                    describe("Convey(" + part + ", " + feeder + ", " + zone +
                                 ")",
                             {},
                             belt),
                    m_conveyor,
                    part,
                    feeder,
                    zone);
            }
        }
    }
    for (std::unique_ptr<robotik::RobotSession> const& robot :
         p_simulation.robots())
    {
        addArmSkills(p_simulation, *robot);
    }
    addPlanning(p_simulation);
}

void ConveyorMission::addArmSkills(robotik::Simulation& p_simulation,
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
    robotik::SkillScheduler& skills = p_simulation.skills();
    skills.add<ReleaseSkill>(describe(scope + "Release", id, cup));
    for (std::string const& part : m_parts)
    {
        skills.add<HoverSkill>(describe(scope + "Hover(" + part + ")", id, arm),
                               part,
                               Length(kHover));
        skills.add<HoverSkill>(describe(scope + "Reach(" + part + ")", id, arm),
                               part,
                               Length(kContact));
        robotik::SkillDescription grasp =
            describe(scope + "Grasp(" + part + ")", id, cup);
        grasp.wait = false;
        skills.add<GraspSkill>(std::move(grasp), part);
    }
    for (std::string const& bin : m_bins)
    {
        skills.add<HoverSkill>(describe(scope + "Over(" + bin + ")", id, arm),
                               bin,
                               Length(kOverBin));
    }
    // Back over a pick zone, where the next part arrives.
    for (std::string const& zone : m_zones)
    {
        skills.add<HoverSkill>(describe(scope + "Ready(" + zone + ")", id, arm),
                               zone,
                               Length(kPartHeight + kHover));
    }
}

/// @brief True when the believed centre of @p_object lies in the footprint
/// of @p_place (shared beliefs: same frame for both).
static bool onFootprint(robotik::WorldObject const& p_object,
                        robotik::WorldObject const& p_place)
{
    return std::abs(p_object.position.x - p_place.position.x) <
               0.5 * p_place.size.x &&
           std::abs(p_object.position.y - p_place.position.y) <
               0.5 * p_place.size.y;
}

/// @brief An arm of the cell holds @p_part.
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

/// @brief at(part, place) on the beliefs: on its footprint, not held. A
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

void ConveyorMission::addPlanning(robotik::Simulation& p_simulation)
{
    using robotik::Args;
    using robotik::RobotContext;
    using robotik::Truth;

    robotik::PlanningDomain& domain = p_simulation.planning();

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

void ConveyorMission::reset(robotik::Simulation& /*p_simulation*/,
                            robotik::Seed /*p_seed*/)
{
    m_conveyor.reset();
    m_overlap = 0.0;
}

void ConveyorMission::step(robotik::Simulation& p_simulation, Seconds p_dt)
{
    m_conveyor.step(p_simulation, p_dt);

    // Both arms busy: a skill of each robot is running.
    robotik::SkillScheduler const& skills = p_simulation.skills();
    std::vector<std::string> busy;
    for (robotik::SkillId id = 0; id < skills.size(); ++id)
    {
        std::string const& robot = skills.description(id).robot;
        if (!robot.empty() &&
            skills.state(id) == robotik::SkillState::Running &&
            std::find(busy.begin(), busy.end(), robot) == busy.end())
        {
            busy.push_back(robot);
        }
    }
    if (busy.size() >= 2u && !p_simulation.finished())
    {
        m_overlap += p_dt.value();
    }
}

void ConveyorMission::measure(robotik::Simulation const& /*p_simulation*/,
                              robotik::Metrics& p_metrics) const
{
    p_metrics.set("dual_arm.overlap", m_overlap);
}

} // namespace dual_arm
