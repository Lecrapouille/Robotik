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
#include "Robotik/Runtime/Simulation.hpp"

#include <cmath>

void PainterMission::setup(robotik::Simulation& p_simulation, robotik::SceneView* p_view)
{
    // The easel and the canvas are entities of painting_lab.yaml: the
    // simulation builds them. The mission brings the gestures.
    robotik::ResourceManager const& resources = p_simulation.robot().resources();
    robotik::ActuatorSet const& actuators = p_simulation.robot().actuators();
    std::vector<robotik::ResourceRequirement> arm;
    if (auto const* group = actuators.first<robotik::JointGroup>())
    {
        arm.push_back(resources.require(group->name()));
    }
    auto describe = [&](std::string p_name) {
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
    }
    skills.add<painter::MarkSkill>(describe("mark"), m_stroke, std::move(strokes));
    skills.add<painter::RinseSkill>(describe("rinse"), m_stroke);
    painter::registerNodes(p_simulation.factory(), skills);

    m_view = p_view;
    if (p_view != nullptr)
    {
        compages::world::Entity const canvas = p_simulation.entity(painter::kCanvas);
        if (!canvas)
        {
            throw std::runtime_error("The painter world needs a 'canvas' entity");
        }
        p_view->openCanvas(canvas, 0.40, 0.50);
    }
}

void PainterMission::step(robotik::Simulation& p_simulation, robotik::Seconds /*p_dt*/)
{
    if (m_view == nullptr || !m_stroke.painting)
    {
        return;
    }
    robotik::Robot const& robot = p_simulation.robot();
    if (robot.tool().empty())
    {
        return;
    }
    // Box centre frame: the painted face is the +X side, half the thickness out.
    robotik::Pose const board = p_simulation.pose(painter::kCanvas);
    robotik::Vector3 const local = board.inverse() * robot.worldPose(robot.tool()).position;
    constexpr double kFace = 0.01;
    if (local.x < kFace || local.x > kFace + 0.05 || std::abs(local.y) > 0.19 ||
        std::abs(local.z) > 0.24)
    {
        return;
    }
    float red = 0.0f;
    float green = 0.0f;
    float blue = 0.0f;
    painter::colorRgb(m_stroke.color, red, green, blue);
    m_view->ink(local.y, local.z, red, green, blue);
}
