// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "TrayMission.hpp"

#include "ArmSkills.hpp"
#include "TraySkills.hpp"

#include "Robotik/Perception/WorldModel.hpp"
#include "Robotik/Planning/PlanningDomain.hpp"
#include "Robotik/Runtime/Metrics.hpp"
#include "Robotik/Runtime/RobotContext.hpp"
#include "Robotik/Runtime/Simulation.hpp"

#include <algorithm>
#include <cmath>

namespace dual_arm
{

/// Height of the cup over the tray before the grasp (m).
static constexpr double kHover = 0.08;
/// Height of the cup over the tray to grasp it: the weld holds it as it is.
static constexpr double kContact = 0.012;
/// The tray goes up, and back down, by this much (m).
static constexpr double kLift = 0.20;
/// The cup goes up after the release (m).
static constexpr double kRetreat = 0.10;
/// Height of the tool above a glass it visits (m).
static constexpr double kVisitClearance = 0.05;
/// The tool counts as above the glass within this distance (m).
static constexpr double kVisitTolerance = 0.02;
/// Times of the smooth moves with the tray (s): slow enough for the glasses.
static constexpr double kRaiseTime = 3.0;
static constexpr double kCarryTime = 10.0;
static constexpr double kLowerTime = 3.0;
static constexpr double kRetreatTime = 1.5;
static constexpr int kPriority = 100;
/// Joint servo gains of the arms: stiff enough to carry the tray and its
/// glasses (0.7 kg) without sagging (the gravity compensation of the drive
/// does not know the payload).
static constexpr double kStiffKp = 800.0;
static constexpr double kStiffKd = 50.0;

/// A skill named @p_name on robot @p_robot holding @p_resources.
static robotik::SkillDescription describe(std::string p_name,
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

void TrayMission::setup(robotik::Simulation& p_simulation, robotik::SceneView* /*p_view*/)
{
    for (robotik::EntitySpec const& entity : p_simulation.scenario().world.entities)
    {
        if (entity.kind == "glass")
        {
            m_glasses.push_back(entity.id);
        }
        else if (entity.kind == "tray")
        {
            m_trays.push_back(entity.id);
        }
        else if (entity.kind == "pad")
        {
            m_pads.push_back(entity.id);
        }
    }
    // An arm with a cup carries; an arm without one visits.
    for (std::unique_ptr<robotik::RobotSession> const& robot : p_simulation.robots())
    {
        robotik::JointSet& joints = robot->joints();
        for (robotik::JointId id = 0; id < joints.size(); ++id)
        {
            joints.gains(id).kp = kStiffKp;
            joints.gains(id).kd = kStiffKd;
        }
        if (cupOf(*robot) != nullptr)
        {
            addCarrierSkills(p_simulation, *robot);
        }
        else
        {
            addVisitorSkills(p_simulation, *robot);
        }
    }
    addPlanning(p_simulation);
}

void TrayMission::addCarrierSkills(robotik::Simulation& p_simulation, robotik::Robot const& p_robot)
{
    robotik::ResourceManager const& resources = p_robot.resources();
    std::vector<robotik::ResourceRequirement> arm;
    if (auto const* group = p_robot.actuators().first<robotik::JointGroup>())
    {
        arm.push_back(resources.require(group->name()));
    }
    std::vector<robotik::ResourceRequirement> const cup = { resources.require(cupOf(p_robot)->name()) };

    std::string const& id = p_robot.id();
    std::string const scope = id + ".";
    robotik::SkillScheduler& skills = p_simulation.skills();
    for (std::string const& tray : m_trays)
    {
        skills.add<HoverSkill>(describe(scope + "Hover(" + tray + ")", id, arm), tray, Length(kHover));
        skills.add<HoverSkill>(describe(scope + "Reach(" + tray + ")", id, arm), tray, Length(kContact));
        robotik::SkillDescription grasp = describe(scope + "Grasp(" + tray + ")", id, cup);
        grasp.wait = false;
        skills.add<GraspSkill>(std::move(grasp), tray);
    }
    for (std::string const& pad : m_pads)
    {
        skills.add<PathSkill>(describe(scope + "Carry(" + pad + ")", id, arm), pad, Seconds(kCarryTime));
    }
    skills.add<PathSkill>(describe(scope + "Raise", id, arm), robotik::Vector3(0.0, 0.0, kLift),
                          Seconds(kRaiseTime));
    skills.add<PathSkill>(describe(scope + "Lower", id, arm), robotik::Vector3(0.0, 0.0, -kLift),
                          Seconds(kLowerTime));
    skills.add<ReleaseSkill>(describe(scope + "Release", id, cup));
    skills.add<PathSkill>(describe(scope + "Retreat", id, arm), robotik::Vector3(0.0, 0.0, kRetreat),
                          Seconds(kRetreatTime));
}

void TrayMission::addVisitorSkills(robotik::Simulation& p_simulation, robotik::Robot const& p_robot)
{
    std::vector<robotik::ResourceRequirement> arm;
    if (auto const* group = p_robot.actuators().first<robotik::JointGroup>())
    {
        arm.push_back(p_robot.resources().require(group->name()));
    }
    std::string const& id = p_robot.id();
    for (std::string const& tray : m_trays)
    {
        for (std::string const& glass : m_glasses)
        {
            p_simulation.skills().add<VisitSkill>(
                describe(id + ".Visit(" + glass + ", " + tray + ")", id, arm), glass, tray,
                Length(kVisitClearance));
        }
    }
}

/// @brief The believed centre of @p_a lies in the footprint of @p_b.
static bool overFootprint(robotik::WorldObject const& p_a, robotik::WorldObject const& p_b)
{
    return std::abs(p_a.position.x - p_b.position.x) < 0.5 * p_b.size.x &&
           std::abs(p_a.position.y - p_b.position.y) < 0.5 * p_b.size.y;
}

/// @brief An arm of the cell holds @p_object.
static bool heldByAnyone(robotik::RobotContext const& p_context, std::string const& p_object)
{
    return std::any_of(p_context.team.begin(), p_context.team.end(),
                       [&](robotik::Robot const* p_robot) { return heldObject(*p_robot) == p_object; });
}

void TrayMission::addPlanning(robotik::Simulation& p_simulation)
{
    using robotik::Args;
    using robotik::RobotContext;
    using robotik::Truth;

    robotik::PlanningDomain& domain = p_simulation.planning();

    // --- State, on the shared beliefs and what each arm measures -----------
    domain.observe("free",
                   [](RobotContext const& p_context, Args p_args)
                   {
                       robotik::Robot const* arm = teammate(p_context, p_args[0]);
                       return arm == nullptr ? Truth::Unknown : robotik::truth(heldObject(*arm).empty());
                   });
    domain.observe("holding",
                   [](RobotContext const& p_context, Args p_args)
                   {
                       robotik::Robot const* arm = teammate(p_context, p_args[0]);
                       return arm == nullptr ? Truth::Unknown
                                             : robotik::truth(heldObject(*arm) == p_args[1]);
                   });
    // Resting: on the footprint of the stand, low, not held.
    domain.observe("rests",
                   [](RobotContext const& p_context, Args p_args)
                   {
                       robotik::WorldObject const* tray = p_context.world_model.find(p_args[0]);
                       robotik::WorldObject const* pad = p_context.world_model.find(p_args[1]);
                       if (tray == nullptr || pad == nullptr)
                       {
                           return Truth::Unknown;
                       }
                       bool const low = tray->position.z < pad->top() + tray->size.z;
                       return robotik::truth(!heldByAnyone(p_context, p_args[0]) && low &&
                                             overFootprint(*tray, *pad));
                   });
    domain.observe("above",
                   [](RobotContext const& p_context, Args p_args)
                   {
                       robotik::WorldObject const* tray = p_context.world_model.find(p_args[0]);
                       robotik::WorldObject const* pad = p_context.world_model.find(p_args[1]);
                       if (tray == nullptr || pad == nullptr)
                       {
                           return Truth::Unknown;
                       }
                       return robotik::truth(heldByAnyone(p_context, p_args[0]) &&
                                             overFootprint(*tray, *pad));
                   });
    // A token of the plan: nothing to see outside of a carry.
    domain.observe("in_motion", [](RobotContext const&, Args) { return Truth::False; });
    domain.observe("on",
                   [](RobotContext const& p_context, Args p_args)
                   {
                       robotik::WorldObject const* glass = p_context.world_model.find(p_args[0]);
                       robotik::WorldObject const* tray = p_context.world_model.find(p_args[1]);
                       if (glass == nullptr || tray == nullptr)
                       {
                           return Truth::Unknown;
                       }
                       return robotik::truth(glass->position.z > tray->position.z &&
                                             overFootprint(*glass, *tray));
                   });
    domain.observe("visited", [this](RobotContext const&, Args p_args)
                   { return robotik::truth(m_visited.contains(p_args[0])); });

    // --- Actions, as skills of each robot ----------------------------------
    domain.bind("lift", { "{0}.Hover({1})", "{0}.Reach({1})", "{0}.Grasp({1})", "{0}.Raise" });
    domain.bind("carry", { "{0}.Carry({3})" });
    domain.bind("visit", { "{0}.Visit({1}, {2})" });
    domain.bind("set_down", { "{0}.Lower", "{0}.Release", "{0}.Retreat" });
}

void TrayMission::reset(robotik::Simulation& /*p_simulation*/, robotik::Seed /*p_seed*/)
{
    m_visited.clear();
    m_overlap = 0.0;
}

void TrayMission::step(robotik::Simulation& p_simulation, Seconds p_dt)
{
    // A tool above a glass, on the beliefs and the measured tool pose.
    robotik::WorldModel const& beliefs = p_simulation.worldModel();
    for (std::unique_ptr<robotik::RobotSession> const& robot : p_simulation.robots())
    {
        robotik::Vector3 const tool =
            p_simulation.beliefFrame().inverse() * robot->worldPose(robot->tool()).position;
        for (std::string const& id : m_glasses)
        {
            robotik::WorldObject const* glass = beliefs.find(id);
            if (glass == nullptr)
            {
                continue;
            }
            robotik::Vector3 const point(glass->position.x, glass->position.y,
                                         glass->top() + kVisitClearance);
            if ((tool - point).norm() < kVisitTolerance)
            {
                m_visited.insert(id);
            }
        }
    }

    // Both arms busy: a skill of each robot is running.
    robotik::SkillScheduler const& skills = p_simulation.skills();
    std::set<std::string> busy;
    for (robotik::SkillId id = 0; id < skills.size(); ++id)
    {
        if (!skills.description(id).robot.empty() && skills.state(id) == robotik::SkillState::Running)
        {
            busy.insert(skills.description(id).robot);
        }
    }
    if (busy.size() >= 2u && !p_simulation.finished())
    {
        m_overlap += p_dt.value();
    }
}

void TrayMission::measure(robotik::Simulation const& /*p_simulation*/,
                          robotik::Metrics& p_metrics) const
{
    p_metrics.set("dual_arm.overlap", m_overlap);
    p_metrics.set("dual_arm.visited", static_cast<double>(m_visited.size()));
}

} // namespace dual_arm
