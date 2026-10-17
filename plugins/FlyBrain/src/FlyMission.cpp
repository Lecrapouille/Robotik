// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "FlyMission.hpp"

#include "Robotik/Assets/AssetResolver.hpp"
#include "Robotik/Backends/SceneView.hpp"
#include "Robotik/ECS/ObjectComponents.hpp"
#include "Robotik/Robot/Robot.hpp"
#include "Robotik/Runtime/Simulation.hpp"
#include "Robotik/Sensors/Image.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace
{

constexpr std::size_t TRAIL = 240;

compages::core::Vector3f toFloat(robotik::Vector3 const& p_v)
{
    return { static_cast<float>(p_v.x), static_cast<float>(p_v.y), static_cast<float>(p_v.z) };
}

compages::core::Quatf toFloat(robotik::Quaternion const& p_q)
{
    return { static_cast<float>(p_q.w),
             static_cast<float>(p_q.x),
             static_cast<float>(p_q.y),
             static_cast<float>(p_q.z) };
}

void poseJoint(robotik::JointSet& p_joints, std::string const& p_name, double p_angle)
{
    robotik::JointId const id = p_joints.find(p_name);
    if (id != robotik::NO_JOINT)
    {
        p_joints.measure(id, p_angle, 0.0);
    }
}

robotik::ecs::Shape decoration(std::string p_name,
                                     robotik::Vector3 const& p_size,
                                     std::array<float, 3> const& p_color)
{
    robotik::ecs::Shape object;
    object.name = std::move(p_name);
    object.size = { Length(p_size.x), Length(p_size.y), Length(p_size.z) };
    object.color = p_color;
    return object;
}

} // namespace

// --- Backend ----------------------------------------------------------------

FlyBackend::FlyBackend(FlyEnvironment& p_environment) : m_environment(p_environment)
{
}

void FlyBackend::restart()
{
    m_command = {};
    m_lag = 0.0;
    m_return = 0.0;
    m_done = false;
}

void FlyBackend::reset(robotik::Robot& p_robot)
{
    pose(p_robot);
}

void FlyBackend::step(robotik::Robot& p_robot, Seconds p_dt)
{
    double const dt = m_environment.dt();
    m_lag += p_dt.value();
    // The fly model keeps its own step; the host step should be the same.
    while (!m_done && m_lag >= dt - 1e-9)
    {
        std::array<float, FlyAction::SIZE> action{};
        m_command.write(action);
        robotik::StepResult const result = m_environment.step(action, m_observation);
        m_return += static_cast<double>(result.reward);
        m_done = result.done();
        m_lag -= dt;
    }
    pose(p_robot);
}

void FlyBackend::pose(robotik::Robot& p_robot) const
{
    FlySnapshot const& snapshot = m_environment.snapshot();
    FlyPosture const& posture = snapshot.posture;
    robotik::JointSet& joints = p_robot.joints();

    // Left and right share the posture: the URDF mirrors the signs.
    for (char const* leg : FLY_LEG_NAMES)
    {
        std::string const name(leg);
        poseJoint(joints, name + "_coxa_pitch", posture.coxa_pitch);
        poseJoint(joints, name + "_coxa_roll", 0.0);
        poseJoint(joints, name + "_coxa_yaw", 0.0);
        poseJoint(joints, name + "_femur_pitch", posture.femur_pitch);
        poseJoint(joints, name + "_femur_roll", 0.0);
        poseJoint(joints, name + "_tibia_pitch", posture.tibia_pitch);
        poseJoint(joints, name + "_tarsus_pitch", posture.tarsus_pitch);
    }
    for (char const* side : { "L", "R" })
    {
        std::string const wing = std::string("wing_") + side;
        poseJoint(joints, wing + "_sweep", posture.wing_sweep);
        poseJoint(joints, wing + "_deviation", posture.wing_deviation);
        poseJoint(joints, wing + "_pitch", posture.wing_pitch);
        poseJoint(joints, std::string("haltere_") + side + "_flap", posture.haltere);
    }
    poseJoint(joints, "neck_yaw", posture.neck_yaw);
    poseJoint(joints, "neck_pitch", posture.neck_pitch);
    poseJoint(joints, "neck_roll", 0.0);
    poseJoint(joints, "abdomen_pitch", posture.abdomen_pitch);

    robotik::Pose pose;
    pose.position = robotik::Vector3(snapshot.plant.x, snapshot.plant.y, snapshot.plant.z);
    pose.rotation = robotik::rpy(Radians(0.0), Radians(0.0), Radians(snapshot.plant.yaw));
    p_robot.measureBase(robotik::BaseState{ pose, {} });
}

// --- Mission ----------------------------------------------------------------

void FlyMission::setup(robotik::Simulation& p_simulation, robotik::SceneView* p_view)
{
    m_simulation = &p_simulation;
    // The FlyWire tables sit in the pack of the fly body (the plugin's own).
    if (robotik::AssetPack const* pack = p_simulation.scenario().assets.find("robotik.fly"))
    {
        m_tables = pack->root / "flywire";
    }
    FlyScenario scenario = FlyScenario::from(p_simulation.scenario());
    m_dt = scenario.dt;
    m_kind = scenario.agent == "reflex" ? FlyBrain::Kind::Reflex : FlyBrain::Kind::Shiu;
    float const cruise = static_cast<float>(scenario.target.z);
    robotik::Vector3 const arena = scenario.arena;
    m_environment = std::make_unique<FlyEnvironment>(std::move(scenario), 0u);
    m_brain = std::make_unique<FlyBrain>(m_kind, cruise);

    auto backend = std::make_unique<FlyBackend>(*m_environment);
    m_backend = backend.get();
    p_simulation.robot().connect(std::move(backend));

    if (p_view == nullptr)
    {
        return;
    }
    // A meadow under the arena, then the beams of the eyes and the trail.
    robotik::Image meadow(2u, 2u, robotik::PixelFormat::RGB8);
    for (std::size_t i = 0; i < meadow.bytes().size(); i += 3u)
    {
        meadow.bytes()[i] = 140;
        meadow.bytes()[i + 1u] = 158;
        meadow.bytes()[i + 2u] = 97;
    }
    double const extent = 2.0 * (std::max(arena.x, arena.y) + 2.0);
    p_view->ground(meadow, extent, extent);

    std::array<std::array<float, 3>, 3> const colors = { { { 0.95f, 0.75f, 0.2f },
                                                           { 0.95f, 0.25f, 0.2f },
                                                           { 0.25f, 0.45f, 1.0f } } };
    char const* names[3] = { "ray_left", "ray_center", "ray_right" };
    for (std::size_t ray = 0; ray < 3u; ++ray)
    {
        // Unit length along x, stretched toward the hit at each step.
        m_beams[ray] = p_simulation.decorate(
            decoration(names[ray], robotik::Vector3(1.0, 0.02, 0.02), colors[ray]));
    }
    for (std::size_t i = 0; i < TRAIL; ++i)
    {
        m_trail.push_back(p_simulation.decorate(decoration(
            "trail_" + std::to_string(i), robotik::Vector3(0.04, 0.04, 0.04), { 0.95f, 0.95f, 0.9f })));
    }
}

void FlyMission::reset(robotik::Simulation& p_simulation, robotik::Seed p_seed)
{
    m_seed = p_seed;
    m_collided = false;
    m_brain->reset(p_seed);
    m_environment->reset(p_seed, m_backend->observation());
    m_backend->restart();

    // The blocks shift a little at each episode: the props follow.
    std::span<FlyBox const> const blocks = m_environment->obstacles();
    std::size_t index = 0;
    for (robotik::EntitySpec const& entity : p_simulation.scenario().world.entities)
    {
        if (entity.type == robotik::EntitySpec::Type::Prop &&
            entity.id.starts_with("obstacle") && index < blocks.size())
        {
            p_simulation.entity(entity.id).position(toFloat(blocks[index].position));
            ++index;
        }
    }

    // Puts the body where the episode starts, then the first decision: the
    // first step flies with it, as in the standalone loop.
    FlyPlant const& plant = m_environment->snapshot().plant;
    p_simulation.robot().startPose(
        robotik::Pose{ robotik::Vector3(plant.x, plant.y, plant.z),
                       robotik::rpy(Radians(0.0), Radians(0.0), Radians(plant.yaw)) });
    decide();
    drawBeamsAndTrail();
}

void FlyMission::step(robotik::Simulation& /*p_simulation*/, Seconds /*p_dt*/)
{
    m_collided = m_collided || m_environment->snapshot().collided;
    if (!m_backend->done())
    {
        decide();
    }
    drawBeamsAndTrail();
}

void FlyMission::decide()
{
    m_action = m_brain->update(FlyObservation::from(m_backend->observation()), Seconds(m_dt));
    m_backend->command(m_action);
}

void FlyMission::drawBeamsAndTrail()
{
    if (!m_beams[0])
    {
        return;
    }
    FlyVision const& vision = m_environment->snapshot().vision;
    for (std::size_t ray = 0; ray < 3u; ++ray)
    {
        robotik::Vector3 const from = vision.origins[ray];
        robotik::Vector3 const span = vision.ends[ray] - from;
        double const length = robotik::norm(span);
        compages::world::Entity beam = m_beams[ray];
        if (length < 1e-6)
        {
            beam.position(0.0f, 0.0f, -20.0f);
            continue;
        }
        robotik::Vector3 const along = span * (1.0 / length);
        robotik::Vector3 const axis(0.0, -along.z, along.y); // x cross along
        double const angle = std::acos(std::clamp(along.x, -1.0, 1.0));
        robotik::Quaternion const turn =
            robotik::norm(axis) < 1e-9
                ? (along.x > 0.0 ? robotik::Quaternion{}
                                 : robotik::axisAngle(robotik::Vector3(0.0, 0.0, 1.0),
                                                      Radians(3.14159265358979323846)))
                : robotik::axisAngle(axis, Radians(angle));
        beam.position(toFloat(from + span * 0.5))
            .rotation(toFloat(turn))
            .scale(static_cast<float>(length), 1.0f, 1.0f);
    }
    std::span<robotik::Vector3 const> const trail = m_environment->trail();
    for (std::size_t i = 0; i < m_trail.size(); ++i)
    {
        compages::world::Entity crumb = m_trail[i];
        if (i < trail.size())
        {
            crumb.position(toFloat(trail[i]));
        }
        else
        {
            crumb.position(0.0f, 0.0f, -20.0f);
        }
    }
}

void FlyMission::measure(robotik::Simulation const& /*p_simulation*/,
                         robotik::Metrics& p_metrics) const
{
    FlySnapshot const& snapshot = m_environment->snapshot();
    std::span<float const> const observation = m_backend->observation();
    p_metrics.set("fly.reached", snapshot.reached ? 1.0 : 0.0);
    p_metrics.set("fly.collided", m_collided ? 1.0 : 0.0);
    p_metrics.set("fly.return", m_backend->episodeReturn());
    p_metrics.set("fly.steps", static_cast<double>(snapshot.steps));
    p_metrics.set("fly.target_distance", static_cast<double>(observation[8]));
    p_metrics.set("fly.neurons", static_cast<double>(m_brain->neurons()));
}

robotik::Status FlyMission::status(robotik::Simulation const& /*p_simulation*/) const
{
    if (m_environment->snapshot().reached)
    {
        return robotik::Status::SUCCESS;
    }
    return m_backend->done() ? robotik::Status::FAILURE : robotik::Status::RUNNING;
}

std::unique_ptr<FlyBrain> FlyMission::buildBrain(bool p_connectome,
                                                 float p_cruise,
                                                 std::filesystem::path const& p_tables)
{
    auto brain = std::make_unique<FlyBrain>(FlyBrain::Kind::Shiu, p_cruise);
    if (p_connectome)
    {
        brain->loadConnectome(p_tables / "edges.csv", p_tables / "binding.txt");
    }
    return brain;
}

float FlyMission::cruise() const
{
    return static_cast<float>(m_environment->scenario().target.z);
}

void FlyMission::installBrain(std::unique_ptr<FlyBrain> p_brain)
{
    m_brain = std::move(p_brain);
    m_kind = FlyBrain::Kind::Shiu;
    if (m_simulation != nullptr)
    {
        m_simulation->reset(m_seed);
    }
}

void FlyMission::useConnectome(bool p_connectome)
{
    installBrain(buildBrain(p_connectome, cruise(), m_tables));
}
