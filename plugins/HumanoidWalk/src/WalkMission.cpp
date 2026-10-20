// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "WalkMission.hpp"
#include "WalkSkills.hpp"

#include "Robotik/Robot/Backends/MujocoBackend.hpp"
#include "Robotik/Robot/Backends/SceneView.hpp"
#include "Robotik/Task/Metrics.hpp"
#include "Robotik/Task/Cell.hpp"
#include "Robotik/Common/Config.hpp"
#include "Robotik/Robot/Sensors/Image.hpp"

#include <cmath>
#include <limits>
#include <numbers>

/// Physics step of the policy training (s).
#define PHYSICS_STEP_S 0.001
/// IMU of the policy: on the pelvis, at the origin of the base link.
#define IMU_NAME "imu"
/// Link whose ground truth pose feeds the metrics.
#define BASE_LINK "base_link"
/// Skill priority, as the other plugins.
#define SKILL_PRIORITY 100
/// The walked path is sampled at this period (s) for the circle fit.
#define PATH_PERIOD_S 0.05
/// Painted ground in the simulator: side and tile (m).
#define GROUND_SIZE_M 12u
#define GROUND_TILE_M 1u

robotik::Seconds WalkMission::physicsStep()
{
    return robotik::Seconds(PHYSICS_STEP_S);
}

// ----------------------------------------------------------------------------
void WalkMission::setup(robotik::Cell& p_cell,
                        robotik::SceneView* p_view)
{
    robotik::CellDescription const& scenario = p_cell.description();
    m_base = robotik::firstRobot(scenario.world).id + "." + BASE_LINK;
    std::filesystem::path const& urdf = robotik::firstRobot(scenario.world).model;
    WalkContract contract =
        WalkContract::load(urdf.parent_path() / "walk.yaml");

    // A free base on a floor, no gravity compensation, the rotor inertia.
    robotik::MujocoOptions options;
    options.floating_base = true;
    options.floor = true;
    options.floor_friction = scenario.world.floor_friction;
    options.timestep = physicsStep();
    options.gravity_compensation = false;
    for (std::size_t i = 0; i < contract.joints.size(); ++i)
    {
        options.armature.emplace(contract.joints[i], contract.armature[i]);
    }
    auto physics = std::make_unique<robotik::MujocoBackend>(std::move(options));
    (void)physics->load(urdf);
    robotik::RobotSession& robot = p_cell.robot();
    robot.connect(std::move(physics));

    // The PD gains of the policy, no slew: the policy targets are smooth.
    WalkBody body{ {}, IMU_NAME };
    robotik::JointSet& joints = robot.joints();
    for (std::size_t i = 0; i < contract.joints.size(); ++i)
    {
        robotik::JointId const id = joints.require(contract.joints[i]);
        robotik::JointGains& gains = joints.gains(id);
        gains.kp = contract.kp[i];
        gains.kd = contract.kd[i];
        gains.speed_ratio = std::numeric_limits<double>::infinity();
        // Damping mode (velocity loop toward zero) after a failure.
        gains.velocity_kp = contract.kd[i];
        body.joints.push_back(id);
    }

    robotik::ConfigNode const& parameters = scenario.parameters;
    m_command = { parameters.number("vx", 0.3),
                  parameters.number("vy", 0.0),
                  parameters.number("yaw_rate", 0.3) };
    robotik::Seconds const stand(parameters.number("stand_duration", 1.0));
    robotik::Seconds const walk(parameters.number("walk_duration", 25.0));
    robotik::Seconds const period(PHYSICS_STEP_S * contract.decimation);
    std::vector<double> const posture = contract.default_angle;

    std::filesystem::path const model = urdf.parent_path() / "walk.onnx";
    m_policy = std::make_unique<WalkPolicy>(model, std::move(contract));

    robotik::SkillDescription describe;
    describe.priority = SKILL_PRIORITY;
    describe.resources = { robot.resources().require("body") };
    describe.name = "Stand";
    p_cell.skills().add<StandSkill>(describe, body, posture, stand);
    describe.name = "Walk";
    m_walk = p_cell.skills().add<WalkSkill>(
        describe, body, *m_policy, m_command, walk, period);

    if (p_view != nullptr)
    {
        paintGround(*p_view);
    }
}

// ----------------------------------------------------------------------------
void WalkMission::paintGround(robotik::SceneView& p_view)
{
    // Grey tiles of one metre: the circle of one metre is easy to read.
    constexpr std::uint32_t TILES = GROUND_SIZE_M / GROUND_TILE_M;
    constexpr std::uint32_t PIXELS = 8u;
    robotik::Image ground(
        TILES * PIXELS, TILES * PIXELS, robotik::PixelFormat::RGB8);
    std::span<std::uint8_t> bytes = ground.bytes();
    for (std::uint32_t row = 0; row < ground.height(); ++row)
    {
        for (std::uint32_t column = 0; column < ground.width(); ++column)
        {
            bool const dark = ((row / PIXELS) + (column / PIXELS)) % 2u == 0u;
            std::uint8_t const level = dark ? 96u : 140u;
            std::size_t const at = 3u * (row * ground.width() + column);
            bytes[at] = level;
            bytes[at + 1u] = level;
            bytes[at + 2u] = static_cast<std::uint8_t>(level + 8u);
        }
    }
    p_view.ground(ground,
                  static_cast<double>(GROUND_SIZE_M),
                  static_cast<double>(GROUND_SIZE_M));
}

// ----------------------------------------------------------------------------
void WalkMission::reset(robotik::Cell& p_cell,
                        robotik::Seed /*p_seed*/)
{
    m_policy->reset();
    robotik::Pose const base = p_cell.pose(m_base);
    m_height = base.position.z;
    m_height_min = base.position.z;
    m_distance = 0.0;
    m_yaw_turned = 0.0;
    m_last_x = base.position.x;
    m_last_y = base.position.y;
    m_last_yaw = robotik::yaw(base.rotation).value();
    m_path.clear();
}

// ----------------------------------------------------------------------------
void WalkMission::step(robotik::Cell& p_cell, robotik::Seconds /*p_dt*/)
{
    robotik::Pose const base = p_cell.pose(m_base);
    m_height = base.position.z;
    m_height_min = std::min(m_height_min, base.position.z);

    bool const walking =
        p_cell.skills().state(m_walk) == robotik::SkillState::Running;
    double const yaw = robotik::yaw(base.rotation).value();
    double const x = base.position.x;
    double const y = base.position.y;
    if (walking)
    {
        m_distance += std::hypot(x - m_last_x, y - m_last_y);
        m_yaw_turned +=
            std::remainder(yaw - m_last_yaw, 2.0 * std::numbers::pi);
        double const now = p_cell.time().value();
        if (m_path.empty() || now >= m_next_sample)
        {
            m_path.push_back({ x, y });
            m_next_sample = now + PATH_PERIOD_S;
        }
    }
    m_last_x = x;
    m_last_y = y;
    m_last_yaw = yaw;
}

// ----------------------------------------------------------------------------
double WalkMission::radius() const
{
    // Mean distance to the centre of the path: the radius once a full turn
    // is walked.
    if (m_path.empty())
    {
        return 0.0;
    }
    double cx = 0.0;
    double cy = 0.0;
    for (auto const& [x, y] : m_path)
    {
        cx += x;
        cy += y;
    }
    cx /= static_cast<double>(m_path.size());
    cy /= static_cast<double>(m_path.size());
    double sum = 0.0;
    for (auto const& [x, y] : m_path)
    {
        sum += std::hypot(x - cx, y - cy);
    }
    return sum / static_cast<double>(m_path.size());
}

double WalkMission::turns() const
{
    return std::abs(m_yaw_turned) / (2.0 * std::numbers::pi);
}

double WalkMission::commandedRadius() const
{
    return std::abs(m_command[2]) > 1e-6 ? std::abs(m_command[0] / m_command[2])
                                         : 0.0;
}

// ----------------------------------------------------------------------------
void WalkMission::measure(robotik::Cell const& /*p_cell*/,
                          robotik::Metrics& p_metrics) const
{
    p_metrics.set("humanoid.height_min", m_height_min);
    p_metrics.set("humanoid.distance", m_distance);
    p_metrics.set("humanoid.turns", turns());
    p_metrics.set("humanoid.radius", radius());
    p_metrics.set("humanoid.commanded_radius", commandedRadius());
}
