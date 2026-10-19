// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "PainterMission.hpp"

#include "PainterScene.hpp"
#include "PainterSkills.hpp"

#include "Robotik/Robot/Actuators.hpp"
#include "Robotik/Robot/Robot.hpp"
#include "Robotik/Package/Simulation.hpp"

#include <algorithm>
#include <cmath>

void PainterMission::setup(robotik::Simulation& p_simulation,
                           robotik::SceneView* p_view)
{
    // The easel and the canvas are entities of painting_lab.world.yaml: the
    // simulation builds them. The mission brings the gestures.
    robotik::ResourceManager const& resources =
        p_simulation.robot().resources();
    robotik::ActuatorSet const& actuators = p_simulation.robot().actuators();
    std::vector<robotik::ResourceRequirement> arm;
    if (auto const* group = actuators.first<robotik::JointGroup>())
    {
        arm.push_back(resources.require(group->name()));
    }
    auto describe = [&](std::string p_name)
    {
        robotik::SkillDescription description;
        description.name = std::move(p_name);
        description.resources = arm;
        description.priority = 100;
        return description;
    };
    robotik::SkillScheduler& skills = p_simulation.skills();
    skills.add<painter::DipSkill>(describe("dip"), m_stroke);
    // The strokes are trajectories of the scenario, drawn in the frame of the
    // easel the world file describes.
    painter::MarkSkill::Strokes strokes;
    for (auto const& entry : p_simulation.scenario().trajectories)
    {
        strokes.emplace(entry.first, p_simulation.waypoints(entry.first));
        m_paths[entry.first] = p_simulation.waypoints(entry.first);
    }
    // How the brush follows a stroke, and the gains of the arm joints: a
    // scenario (or an experiment) tunes them in parameters.
    robotik::ConfigNode const& parameters = p_simulation.scenario().parameters;
    painter::MarkSkill::Settings settings;
    settings.step = parameters.number("stroke_step", settings.step);
    settings.tolerance =
        parameters.number("stroke_tolerance", settings.tolerance);
    skills.add<painter::MarkSkill>(
        describe("mark"), m_stroke, std::move(strokes), settings);
    robotik::ConfigNode const& gains = parameters.at("arm_gains");
    if (gains.isMap())
    {
        robotik::JointSet& joints = p_simulation.robot().joints();
        for (robotik::JointId id = 0; id < joints.size(); ++id)
        {
            robotik::JointGains& gain = joints.gains(id);
            gain.kp = gains.number("kp", gain.kp);
            gain.kd = gains.number("kd", gain.kd);
            gain.speed_ratio = gains.number("speed_ratio", gain.speed_ratio);
        }
    }
    skills.add<painter::RinseSkill>(describe("rinse"), m_stroke);
    painter::registerNodes(p_simulation.factory(), skills);
    if (m_planner)
    {
        registerPlanning(p_simulation);
    }

    m_view = p_view;
    if (p_view != nullptr)
    {
        compages::world::Entity const canvas =
            p_simulation.entity(painter::kCanvas);
        if (!canvas)
        {
            throw std::runtime_error(
                "The painter world needs a 'canvas' entity");
        }
        p_view->openCanvas(canvas, 0.40, 0.50);
    }
}

void PainterMission::reset(robotik::Simulation& /*p_simulation*/,
                           robotik::Seed /*p_seed*/)
{
    m_stroke.brush.clear();
    m_stroke.painted.clear();
    m_deviation = 0.0;
    m_inked = 0;
}

void PainterMission::measure(robotik::Simulation const& /*p_simulation*/,
                             robotik::Metrics& p_metrics) const
{
    p_metrics.set("painter.stroke_deviation", m_deviation);
    p_metrics.set("painter.inked", static_cast<double>(m_inked));
}

// The robot cannot see the paint: what the planner knows of the brush and
// the canvas is what the gestures did (Stroke::brush, Stroke::painted).
void PainterMission::registerPlanning(robotik::Simulation& p_simulation)
{
    using robotik::Args;
    using robotik::RobotContext;
    robotik::PlanningDomain& domain = p_simulation.planning();
    painter::Stroke const& stroke = m_stroke;
    domain.observe("clean",
                   [&stroke](RobotContext const&, Args) noexcept
                   { return robotik::truth(stroke.brush.empty()); });
    domain.observe("loaded",
                   [&stroke](RobotContext const&, Args p_args) noexcept
                   { return robotik::truth(stroke.brush == p_args[0]); });
    domain.observe(
        "painted",
        [&stroke](RobotContext const&, Args p_args)
        { return robotik::truth(stroke.painted.contains(p_args[0])); });
    // The gesture nodes read the colour on the blackboard of the step.
    domain.bind("dip", { "Dip", "Home" }, { { "color", "{0}" } });
    domain.bind("mark", { "Mark", "Home" }, { { "color", "{0}" } });
    domain.bind("rinse", { "Rinse", "Home" }, { { "color", "{0}" } });
}

void PainterMission::step(robotik::Simulation& p_simulation,
                          robotik::Seconds /*p_dt*/)
{
    if (!m_stroke.painting)
    {
        return;
    }
    robotik::Robot const& robot = p_simulation.robot();
    if (robot.tool().empty())
    {
        return;
    }
    // Box centre frame: the painted face is the +X side, half the thickness
    // out.
    robotik::Pose const board = p_simulation.pose(painter::kCanvas);
    robotik::Vector3 const local =
        board.inverse() * robot.worldPose(robot.tool()).position;
    constexpr double kFace = 0.01;
    if (local.x < kFace || local.x > kFace + 0.05 || std::abs(local.y) > 0.19 ||
        std::abs(local.z) > 0.24)
    {
        return;
    }
    // The ink lands here: how far from the straight line of the stroke (its
    // third and fourth waypoints), in the plane of the canvas?
    ++m_inked;
    if (auto const path = m_paths.find(m_stroke.color);
        path != m_paths.end() && path->second.size() >= 4u)
    {
        robotik::Vector3 const a = board.inverse() * path->second[2].position;
        robotik::Vector3 const b = board.inverse() * path->second[3].position;
        double const dy = b.y - a.y;
        double const dz = b.z - a.z;
        double const length2 = dy * dy + dz * dz;
        double const t =
            length2 > 0.0
                ? std::clamp(((local.y - a.y) * dy + (local.z - a.z) * dz) /
                                 length2,
                             0.0,
                             1.0)
                : 0.0;
        m_deviation = std::max(
            m_deviation,
            std::hypot(local.y - (a.y + t * dy), local.z - (a.z + t * dz)));
    }
    if (m_view == nullptr)
    {
        return;
    }
    float red = 0.0f;
    float green = 0.0f;
    float blue = 0.0f;
    painter::colorRgb(m_stroke.color, red, green, blue);
    m_view->ink(local.y, local.z, red, green, blue);
}
