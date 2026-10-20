// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#pragma once

#include "WalkPolicy.hpp"

#include "Robotik/Task/Mission.hpp"
#include "Robotik/Task/Scheduler.hpp"

#include <array>
#include <memory>
#include <string>
#include <vector>

// ****************************************************************************
/// A humanoid walks by a learned policy (LimX HU_D04_01).
///
/// Setup plugs a floating-base MuJoCo backend without gravity compensation
/// (the policy was trained without it), gives every joint the PD gains of the
/// policy and registers two skills on the resource "body":
/// @li @c Stand holds the default posture (parameters.stand_duration);
/// @li @c Walk runs the policy with the command (parameters.vx, vy,
///     yaw_rate) for parameters.walk_duration.
///
/// The policy and its contract (walk.onnx, walk.yaml) sit next to the URDF
/// in the asset pack. Metrics, from the ground truth of the base link:
/// humanoid.height_min (m), humanoid.distance (m, path length in the
/// plane), humanoid.turns (yaw turned while walking, in turns),
/// humanoid.radius (m, mean distance to the centre of the walked path) and
/// humanoid.commanded_radius (m, vx / yaw_rate).
// ****************************************************************************
class WalkMission final: public robotik::Mission
{
public:

    void setup(robotik::Cell& p_cell,
               robotik::SceneView* p_view) override;
    void reset(robotik::Cell& p_cell,
               robotik::Seed p_seed) override;
    void step(robotik::Cell& p_cell, robotik::Seconds p_dt) override;
    void measure(robotik::Cell const& p_cell,
                 robotik::Metrics& p_metrics) const override;

    /// Command of the Walk skill: forward (m/s), lateral (m/s), yaw
    /// rate (rad/s). Starts from the scenario parameters; a panel may change it
    /// while the robot walks.
    [[nodiscard]] std::array<double, 3>& command()
    {
        return m_command;
    }

    /// Base link height now (m), ground truth.
    [[nodiscard]] double height() const
    {
        return m_height;
    }
    [[nodiscard]] double distance() const
    {
        return m_distance;
    }
    [[nodiscard]] double turns() const;
    [[nodiscard]] double radius() const;
    [[nodiscard]] double commandedRadius() const;

    /// Physics step of the backend: the plugin steps the simulation
    /// at this rate so the policy runs exactly every decimation steps.
    [[nodiscard]] static robotik::Seconds physicsStep();

private:

    std::unique_ptr<WalkPolicy> m_policy;
    /// Ground truth reference of the base link: "<robot id>.base_link".
    std::string m_base;
    void paintGround(robotik::SceneView& p_view);

    robotik::SkillId m_walk = robotik::NO_SKILL;
    std::array<double, 3> m_command{};

    // Episode statistics.
    double m_height = 0.0;
    double m_height_min = 0.0;
    double m_distance = 0.0;
    double m_yaw_turned = 0.0;
    double m_last_x = 0.0;
    double m_last_y = 0.0;
    double m_last_yaw = 0.0;
    double m_next_sample = 0.0;
    /// Positions while walking, to fit the circle at the end.
    std::vector<std::array<double, 2>> m_path;
};
