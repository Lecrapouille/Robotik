// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "TrayMission.hpp"

#include "ArmSkills.hpp"
#include "TraySkills.hpp"

#include "Robotik/Task/Perception/WorldModel.hpp"
#include "Robotik/Task/Planning/PlanningDomain.hpp"
#include "Robotik/Task/Metrics.hpp"
#include "Robotik/Task/RobotContext.hpp"
#include "Robotik/Task/Cell.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace dual_arm
{

/// Gap between the cup and the bottom face of the tray to grasp it: the
/// weld holds it as it is (m).
static constexpr double CONTACT = 0.005;
/// The tray goes up, and back down, by this much: clear of the rails (m).
static constexpr double LIFT = 0.05;
/// The cup goes down after the release (m).
static constexpr double RETREAT = 0.04;
/// Force of the drill on the tray, felt by its force sensor (N).
static constexpr double DRILL_FORCE = 5.0;
/// Speed of the drill along its path on the tray (m/s).
static constexpr double DRILL_SPEED = 0.06;
/// Name of the path of the drill among the trajectories of the scenario.
static constexpr char const* DRILL_PATH = "drill";
/// Times of the smooth moves (s): slow enough for the glasses.
static constexpr double APPROACH_TIME = 2.5;
static constexpr double REACH_TIME = 1.0;
static constexpr double RAISE_TIME = 2.0;
static constexpr double CARRY_TIME = 20.0;
static constexpr double LOWER_TIME = 2.0;
static constexpr double RETREAT_TIME = 1.0;
static constexpr int PRIORITY = 100;
/// Joint servo gains of the arms: stiff enough to carry the tray and its
/// glasses (0.7 kg) without sagging (the gravity compensation of the drive
/// does not know the payload).
static constexpr double STIFF_KP = 800.0;
static constexpr double STIFF_KD = 50.0;

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

void TrayMission::setup(robotik::Cell& p_cell,
                        robotik::SceneView* /*p_view*/)
{
    for (robotik::EntitySpec const& entity :
         p_cell.description().world.entities)
    {
        if (entity.kind == "tray")
        {
            m_trays.push_back(entity.id);
        }
        else if (entity.kind == "pad")
        {
            m_pads.push_back(entity.id);
        }
        for (std::string const& workpiece : entity.touches)
        {
            m_touches.push_back("touching(" + entity.id + ", " + workpiece +
                                ")");
        }
    }
    // An arm with a cup carries; an arm without one drills.
    for (std::unique_ptr<robotik::RobotSession> const& robot :
         p_cell.robots())
    {
        robotik::JointSet& joints = robot->joints();
        for (robotik::JointId id = 0; id < joints.size(); ++id)
        {
            joints.gains(id).kp = STIFF_KP;
            joints.gains(id).kd = STIFF_KD;
        }
        if (cupOf(*robot) != nullptr)
        {
            addCarrierSkills(p_cell, *robot);
        }
        else
        {
            addDrillSkills(p_cell, *robot);
        }
    }
    addPlanning(p_cell);
}

void TrayMission::addCarrierSkills(robotik::Cell& p_cell,
                                   robotik::Robot const& p_robot)
{
    robotik::ResourceManager const& resources = p_robot.resources();
    std::vector<robotik::ResourceRequirement> arm;
    if (auto const* group = p_robot.actuators().first<robotik::JointGroup>())
    {
        arm.push_back(resources.require(group->name()));
    }
    std::vector<robotik::ResourceRequirement> const cup = { resources.require(
        cupOf(p_robot)->name()) };

    // The cup points up: it slides under the tray between the rails, at the
    // height it waits at, then rises to its bottom face.
    std::string const& id = p_robot.id();
    std::string const scope = id + ".";
    robotik::SkillScheduler& skills = p_cell.skills();
    for (std::string const& tray : m_trays)
    {
        skills.add<PathSkill>(
            describe(scope + "Approach(" + tray + ")", id, arm),
            tray,
            robotik::Seconds(APPROACH_TIME));
        skills.add<PathSkill>(describe(scope + "Reach(" + tray + ")", id, arm),
                              tray,
                              robotik::Length(CONTACT),
                              robotik::Seconds(REACH_TIME));
        robotik::SkillDescription grasp =
            describe(scope + "Grasp(" + tray + ")", id, cup);
        grasp.wait = false;
        skills.add<GraspSkill>(std::move(grasp), tray);
    }
    for (std::string const& pad : m_pads)
    {
        skills.add<PathSkill>(describe(scope + "Carry(" + pad + ")", id, arm),
                              pad,
                              robotik::Seconds(CARRY_TIME));
    }
    skills.add<PathSkill>(describe(scope + "Raise", id, arm),
                          robotik::Vector3(0.0, 0.0, LIFT),
                          robotik::Seconds(RAISE_TIME));
    skills.add<PathSkill>(describe(scope + "Lower", id, arm),
                          robotik::Vector3(0.0, 0.0, -LIFT),
                          robotik::Seconds(LOWER_TIME));
    skills.add<ReleaseSkill>(describe(scope + "Release", id, cup));
    skills.add<PathSkill>(describe(scope + "Retreat", id, arm),
                          robotik::Vector3(0.0, 0.0, -RETREAT),
                          robotik::Seconds(RETREAT_TIME));
}

void TrayMission::addDrillSkills(robotik::Cell& p_cell,
                                 robotik::Robot const& p_robot)
{
    std::vector<robotik::ResourceRequirement> arm;
    if (auto const* group = p_robot.actuators().first<robotik::JointGroup>())
    {
        arm.push_back(p_robot.resources().require(group->name()));
    }
    // The path is a trajectory of the scenario, drawn in the frame of the tray.
    auto const& trajectories = p_cell.description().trajectories;
    auto const found = trajectories.find(DRILL_PATH);
    if (found == trajectories.end())
    {
        throw std::runtime_error(
            std::string("the scenario has no trajectory '") + DRILL_PATH + "'");
    }
    std::vector<robotik::Vector3> path;
    for (robotik::Pose const& point : found->second.poses)
    {
        path.push_back(point.position);
    }
    std::string const& id = p_robot.id();
    for (std::string const& tray : m_trays)
    {
        p_cell.skills().add<TraceSkill>(
            describe(id + ".Trace(" + tray + ")", id, arm),
            tray,
            path,
            robotik::LinearVelocity(DRILL_SPEED),
            robotik::Force(DRILL_FORCE));
    }
}

/// The believed centre of @p_a lies in the footprint of @p_b.
static bool overFootprint(robotik::WorldObject const& p_a,
                          robotik::WorldObject const& p_b)
{
    return std::abs(p_a.position.x - p_b.position.x) < 0.5 * p_b.size.x &&
           std::abs(p_a.position.y - p_b.position.y) < 0.5 * p_b.size.y;
}

/// An arm of the cell holds @p_object.
static bool heldByAnyone(robotik::RobotContext const& p_context,
                         std::string const& p_object)
{
    return std::ranges::any_of(p_context.team,
                               [&](robotik::Robot const* p_robot)
                               { return heldObject(*p_robot) == p_object; });
}

void TrayMission::addPlanning(robotik::Cell& p_cell)
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
    // Resting: not held, over the pad of a stand (the stands hold it up).
    domain.observe("rests",
                   [](RobotContext const& p_context, Args p_args)
                   {
                       robotik::WorldObject const* tray =
                           p_context.world_model.find(p_args[0]);
                       robotik::WorldObject const* pad =
                           p_context.world_model.find(p_args[1]);
                       if (tray == nullptr || pad == nullptr)
                       {
                           return Truth::Unknown;
                       }
                       return robotik::truth(
                           !heldByAnyone(p_context, p_args[0]) &&
                           overFootprint(*tray, *pad));
                   });
    domain.observe("above",
                   [](RobotContext const& p_context, Args p_args)
                   {
                       robotik::WorldObject const* tray =
                           p_context.world_model.find(p_args[0]);
                       robotik::WorldObject const* pad =
                           p_context.world_model.find(p_args[1]);
                       if (tray == nullptr || pad == nullptr)
                       {
                           return Truth::Unknown;
                       }
                       return robotik::truth(
                           heldByAnyone(p_context, p_args[0]) &&
                           overFootprint(*tray, *pad));
                   });
    // A token of the plan: nothing to see outside of a carry.
    domain.observe("in_motion",
                   [](RobotContext const&, Args) noexcept
                   { return Truth::False; });
    domain.observe("drilled",
                   [this](RobotContext const&, Args p_args)
                   { return robotik::truth(m_drilled.contains(p_args[0])); });

    // --- Actions, as skills of each robot ----------------------------------
    domain.bind("lift",
                { "{0}.Approach({1})",
                  "{0}.Reach({1})",
                  "{0}.Grasp({1})",
                  "{0}.Raise" });
    domain.bind("carry", { "{0}.Carry({3})" });
    domain.bind("drill", { "{0}.Trace({1})" });
    domain.bind("set_down", { "{0}.Lower", "{0}.Release", "{0}.Retreat" });
}

void TrayMission::reset(robotik::Cell& /*p_cell*/,
                        robotik::Seed /*p_seed*/)
{
    m_drilled.clear();
    m_overlap = 0.0;
    m_contact = 0.0;
}

void TrayMission::step(robotik::Cell& p_cell, robotik::Seconds p_dt)
{
    // A tray is drilled once the drill followed its whole path on it.
    robotik::SkillScheduler const& skills = p_cell.skills();
    for (std::string const& tray : m_trays)
    {
        for (std::unique_ptr<robotik::RobotSession> const& robot :
             p_cell.robots())
        {
            robotik::SkillId const trace =
                skills.find(robot->id() + ".Trace(" + tray + ")");
            if (trace != robotik::NO_SKILL &&
                skills.state(trace) == robotik::SkillState::Succeeded)
            {
                m_drilled.insert(tray);
            }
        }
    }

    // A tool on its workpiece: ground truth, for the metric only.
    if (!p_cell.controllerStopped() &&
        std::ranges::any_of(
            m_touches,
            [&](std::string const& p_query)
            { return p_cell.query(p_query).value_or(0.0) > 0.5; }))
    {
        m_contact += p_dt.value();
    }

    // Both arms busy: a skill of each robot is running.
    std::set<std::string, std::less<>> busy;
    for (robotik::SkillId id = 0; id < skills.size(); ++id)
    {
        if (!skills.description(id).robot.empty() &&
            skills.state(id) == robotik::SkillState::Running)
        {
            busy.insert(skills.description(id).robot);
        }
    }
    if (busy.size() >= 2U && !p_cell.controllerStopped())
    {
        m_overlap += p_dt.value();
    }
}

void TrayMission::measure(robotik::Cell const& /*p_cell*/,
                          robotik::Metrics& p_metrics) const
{
    p_metrics.set("dual_arm.overlap", m_overlap);
    p_metrics.set("dual_arm.contact", m_contact);
}

} // namespace dual_arm
