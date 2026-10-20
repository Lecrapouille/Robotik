// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "DuckMission.hpp"
#include "DuckSkills.hpp"

#include "Robotik/Robot/Backends/MujocoBackend.hpp"
#include "Robotik/Task/Metrics.hpp"
#include "Robotik/Task/Cell.hpp"
#include "Robotik/Common/Config.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

/// IMU of the policy: on the trunk, at the origin of trunk_base.
#define IMU_NAME "imu"
/// Link whose ground truth pose feeds the metrics.
#define TRUNK "trunk_base"
/// Skill priority, as the other plugins.
#define SKILL_PRIORITY 100
// ----------------------------------------------------------------------------
void DuckMission::setup(robotik::Cell& p_cell,
                        robotik::SceneView* /*p_view*/)
{
    robotik::CellDescription const& scenario = p_cell.description();
    m_base = scenario.world.robot().id + "." + TRUNK;
    std::filesystem::path const& urdf = scenario.world.robot().model;
    DuckContract contract =
        DuckContract::load(urdf.parent_path() / "walk.yaml");
    m_step = robotik::Seconds(contract.physics_step);

    // A free base on a floor, no gravity compensation. The rotors are far
    // below the default floor, so the given inertia is kept as written. The
    // core builds the physics after setup, with the room of the world.
    robotik::MujocoOptions& options = p_cell.physicsOptions();
    options.floating_base = true;
    options.floor = true;
    options.timestep = m_step;
    options.gravity_compensation = false;
    options.exact_inertia = true;
    for (std::size_t i = 0; i < contract.joints.size(); ++i)
    {
        options.armature.emplace(contract.joints[i], contract.armature[i]);
        options.damping.emplace(contract.joints[i], contract.damping[i]);
    }
    robotik::RobotSession& robot = p_cell.robot();

    // The position gain of the policy, no slew: the policy targets are the
    // whole command. Viscosity is the MuJoCo damping, not this kd (it is 0).
    DuckBody body{ {}, IMU_NAME };
    robotik::JointSet& joints = robot.joints();
    for (std::size_t i = 0; i < contract.joints.size(); ++i)
    {
        robotik::JointId const id = joints.require(contract.joints[i]);
        robotik::JointGains& gains = joints.gains(id);
        gains.kp = contract.kp[i];
        gains.kd = contract.kd[i];
        gains.speed_ratio = std::numeric_limits<double>::infinity();
        gains.velocity_kp = contract.damping[i];
        body.joints.push_back(id);
    }

    robotik::ConfigNode const& parameters = scenario.parameters;
    m_command = { parameters.number("vx", 0.25),
                  parameters.number("vy", 0.0),
                  parameters.number("yaw_rate", 0.0) };
    m_forward = { parameters.number("forward_speed", 0.25), 0.0, 0.0 };
    m_turn = { 0.0, 0.0, parameters.number("turn_rate", 0.8) };
    robotik::Seconds const stand(parameters.number("stand_duration", 0.02));
    robotik::Seconds const walk(parameters.number("walk_duration", 0.0));
    robotik::Seconds const forward(parameters.number("forward_duration", 3.0));
    robotik::Seconds const turn(parameters.number("turn_duration", 2.0));
    robotik::Seconds const period(contract.physics_step * contract.decimation);
    std::vector<double> const posture = contract.default_angle;

    std::filesystem::path const model = urdf.parent_path() / "walk.onnx";
    m_policy = std::make_unique<DuckPolicy>(model, std::move(contract));

    robotik::SkillDescription describe;
    describe.priority = SKILL_PRIORITY;
    describe.resources = { robot.resources().require("body") };
    robotik::SkillScheduler& skills = p_cell.skills();
    describe.name = "Stand";
    skills.add<StandSkill>(describe, body, posture, stand);
    describe.name = "Walk";
    skills.add<WalkSkill>(describe, body, *m_policy, m_command, walk, period);
    describe.name = "Forward";
    skills.add<WalkSkill>(
        describe, body, *m_policy, m_forward, forward, period);
    describe.name = "TurnLeft";
    skills.add<WalkSkill>(describe, body, *m_policy, m_turn, turn, period);
}

// ----------------------------------------------------------------------------
void DuckMission::reset(robotik::Cell& p_cell,
                        robotik::Seed /*p_seed*/)
{
    m_policy->reset();
    robotik::Pose const base = p_cell.pose(m_base);
    m_height = base.position.z;
    m_height_min = base.position.z;
    m_distance = 0.0;
    m_last_x = base.position.x;
    m_last_y = base.position.y;
    m_start_x = base.position.x;
    m_start_y = base.position.y;
}

// ----------------------------------------------------------------------------
void DuckMission::step(robotik::Cell& p_cell, robotik::Seconds /*p_dt*/)
{
    robotik::Pose const base = p_cell.pose(m_base);
    m_height = base.position.z;
    m_height_min = std::min(m_height_min, base.position.z);
    double const x = base.position.x;
    double const y = base.position.y;
    m_distance += std::hypot(x - m_last_x, y - m_last_y);
    m_last_x = x;
    m_last_y = y;
}

// ----------------------------------------------------------------------------
void DuckMission::measure(robotik::Cell const& /*p_cell*/,
                          robotik::Metrics& p_metrics) const
{
    p_metrics.set("duck.height_min", m_height_min);
    p_metrics.set("duck.distance", m_distance);
    p_metrics.set("duck.displacement",
                  std::hypot(m_last_x - m_start_x, m_last_y - m_start_y));
}
