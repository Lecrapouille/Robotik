// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#pragma once

#include "DuckPolicy.hpp"

#include "Robotik/Task/Mission.hpp"

#include <array>
#include <memory>
#include <string>

// ****************************************************************************
/// @brief A Microduck walks by a learned policy (velstand, 14 servos).
///
/// Setup asks the core for a floating base without gravity compensation, at
/// the physics step of the policy, and gives every servo the PD gain and the
/// rotor inertia of the training model. The core then builds the physics
/// with the bodies of the world (the room). The skills share the resource
/// "body":
/// @li @c Stand holds the home posture (parameters.stand_duration);
/// @li @c Walk runs the policy with the command (parameters.vx, vy, yaw_rate)
///     for parameters.walk_duration; a panel may change the command;
/// @li @c Forward and @c TurnLeft run it with a fixed command
///     (parameters.forward_speed, forward_duration, turn_rate, turn_duration).
///
/// Metrics, from the ground truth of the trunk: duck.height_min (m),
/// duck.distance (m, path length in the plane, sway included) and
/// duck.displacement (m, from the start to now, in the plane).
// ****************************************************************************
class DuckMission final: public robotik::Mission
{
public:

    void setup(robotik::Simulation& p_simulation,
               robotik::SceneView* p_view) override;
    void reset(robotik::Simulation& p_simulation,
               robotik::Seed p_seed) override;
    void step(robotik::Simulation& p_simulation, robotik::Seconds p_dt) override;
    void measure(robotik::Simulation const& p_simulation,
                 robotik::Metrics& p_metrics) const override;

    /// @brief Command of the Walk skill: forward (m/s), lateral (m/s), yaw
    /// rate (rad/s). Starts from the scenario parameters; a panel may change it
    /// while the duck walks.
    [[nodiscard]] std::array<double, 3>& command()
    {
        return m_command;
    }

    /// @brief Trunk height now (m), ground truth.
    [[nodiscard]] double height() const
    {
        return m_height;
    }
    [[nodiscard]] double distance() const
    {
        return m_distance;
    }

    /// @brief Physics step of the backend.
    [[nodiscard]] robotik::Seconds physicsStep() const
    {
        return m_step;
    }

private:

    std::unique_ptr<DuckPolicy> m_policy;
    /// Ground truth reference of the trunk: "<robot id>.trunk_base".
    std::string m_base;
    robotik::Seconds m_step{ 0.005 };

    std::array<double, 3> m_command{};
    /// Fixed commands of Forward and TurnLeft.
    std::array<double, 3> m_forward{};
    std::array<double, 3> m_turn{};

    double m_height = 0.0;
    double m_height_min = 0.0;
    double m_distance = 0.0;
    double m_last_x = 0.0;
    double m_last_y = 0.0;
    /// Where the trunk started (m): the displacement is from there.
    double m_start_x = 0.0;
    double m_start_y = 0.0;
};
