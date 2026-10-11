// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "PainterMission.hpp"

#include "PainterScene.hpp"
#include "PainterSkills.hpp"

#include "Robotik/Backends/MujocoBackend.hpp"
#include "Robotik/ECS/ObjectComponents.hpp"
#include "Robotik/Robot/Actuators.hpp"
#include "Robotik/Robot/Robot.hpp"
#include "Robotik/Runtime/Simulation.hpp"

#include <cmath>
#include <optional>

namespace
{

void place(compages::world::Entity p_entity,
           compages::core::Quatf const& p_up,
           robotik::Pose const& p_pose)
{
    compages::core::Quatf const rotation(
        static_cast<float>(p_pose.rotation.w),
        static_cast<float>(p_pose.rotation.x),
        static_cast<float>(p_pose.rotation.y),
        static_cast<float>(p_pose.rotation.z));
    compages::core::Vector3f const at(static_cast<float>(p_pose.position.x),
                                      static_cast<float>(p_pose.position.y),
                                      static_cast<float>(p_pose.position.z));
    p_entity.position(p_up * at).rotation(p_up * rotation);
}

} // namespace

void PainterMission::setup(robotik::Simulation& p_simulation, robotik::SceneView* p_view)
{
    // The canvas is a free body of the simulation, not a link of the easel.
    p_simulation.loadFloor(painter::kFloorFriction);
    std::filesystem::path const easel =
        p_simulation.scenario().assets.model(painter::kEaselModel);
    p_simulation.loadChain(easel,
                           true,
                           painter::easelPose(),
                           painter::kEaselFriction);
    p_simulation.loadBox(painter::kCanvas,
                         { 0.02, 0.40, 0.50 },
                         0.5,
                         { 0.01708, 0.01043, 0.00668 },
                         painter::canvasPose(),
                         painter::kEaselFriction);

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
    skills.add<painter::MarkSkill>(describe("mark"), m_stroke);
    skills.add<painter::RinseSkill>(describe("rinse"), m_stroke);
    painter::registerNodes(p_simulation.factory(), skills);

    m_view = p_view;
    if (p_view == nullptr)
    {
        return;
    }
    compages::world::World& world = p_simulation.robot().world();
    m_easel = p_view->model(world, easel);
    m_up = m_easel.rotation();
    m_canvas = world.entity(painter::kCanvas);
    robotik::ecs::SceneObject canvas;
    canvas.name = painter::kCanvas;
    canvas.size = { robotik::Length(0.02), robotik::Length(0.40), robotik::Length(0.50) };
    canvas.color = { 0.95f, 0.95f, 0.93f };
    m_canvas.set(canvas);
    p_view->object(m_canvas, canvas);
    p_view->openCanvas(m_canvas, 0.40, 0.50);
    m_drawn = true;
    place(m_easel, m_up, painter::easelPose());
    place(m_canvas, m_up, painter::canvasPose());
}

void PainterMission::reset(robotik::Simulation& p_simulation, robotik::Seed /*p_seed*/)
{
    sync(p_simulation);
}

void PainterMission::step(robotik::Simulation& p_simulation, robotik::Seconds /*p_dt*/)
{
    sync(p_simulation);
    if (m_view == nullptr || !m_stroke.painting)
    {
        return;
    }
    auto const* physics =
        dynamic_cast<robotik::MujocoBackend const*>(p_simulation.robot().backend());
    if (physics == nullptr)
    {
        return;
    }
    std::optional<robotik::Pose> const board = physics->bodyPose(painter::kCanvas);
    if (!board)
    {
        return;
    }
    robotik::Robot const& robot = p_simulation.robot();
    if (robot.tool().empty())
    {
        return;
    }
    // Box centre frame: the painted face is the +X side, half the thickness out.
    robotik::Vector3 const local =
        board->inverse() * robot.framePose(robot.tool()).position;
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

void PainterMission::sync(robotik::Simulation& p_simulation)
{
    if (!m_drawn)
    {
        return;
    }
    auto const* physics =
        dynamic_cast<robotik::MujocoBackend const*>(p_simulation.robot().backend());
    if (physics == nullptr)
    {
        return;
    }
    if (std::optional<robotik::Pose> const easel = physics->bodyPose(painter::kEaselRoot))
    {
        place(m_easel, m_up, *easel);
    }
    if (std::optional<robotik::Pose> const canvas = physics->bodyPose(painter::kCanvas))
    {
        place(m_canvas, m_up, *canvas);
    }
}
