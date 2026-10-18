// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#pragma once

#include "DuckPolicy.hpp"

#include "Robotik/Robot/Joints.hpp"
#include "Robotik/Skills/Skill.hpp"

#include <array>
#include <string>
#include <vector>

// ****************************************************************************
/// @brief Joints of the policy, in its order, and the IMU that feeds it.
// ****************************************************************************
struct DuckBody
{
    std::vector<robotik::JointId> joints;
    std::string imu;
};

// ****************************************************************************
/// @brief Holds the home posture of the policy for a while: the duck settles
/// on its feet before walking.
// ****************************************************************************
class StandSkill final: public robotik::Skill
{
public:

    StandSkill(DuckBody p_body,
               std::vector<double> p_posture,
               Seconds p_duration);

    void reset() override;
    robotik::Status tick(robotik::RobotContext& p_context,
                         Seconds p_dt) override;
    void cancel(robotik::RobotContext& p_context) override;

private:

    DuckBody m_body;
    std::vector<double> m_posture;
    Seconds m_duration;
    Seconds m_elapsed{};
};

// ****************************************************************************
/// @brief Walks with a velocity command for a while (forever when the
/// duration is not positive). The command is read at each policy run: a panel
/// may change it while the duck walks.
///
/// The policy runs every decimation physics steps on the IMU and the joint
/// encoders; between two runs the joint PD loops hold its targets. A tilt
/// does not fail the skill: this policy was trained to stand back up. The
/// skill fails only when the IMU is lost, and then leaves the joints in
/// damping mode.
// ****************************************************************************
class WalkSkill final: public robotik::Skill
{
public:

    WalkSkill(DuckBody p_body,
              DuckPolicy& p_policy,
              std::array<double, 3> const& p_command,
              Seconds p_duration,
              Seconds p_period);

    void reset() override;
    robotik::Status tick(robotik::RobotContext& p_context,
                         Seconds p_dt) override;
    void cancel(robotik::RobotContext& p_context) override;

private:

    DuckBody m_body;
    DuckPolicy& m_policy;
    /// Forward (m/s), lateral (m/s), yaw rate (rad/s), owned by the mission.
    std::array<double, 3> const& m_command;
    Seconds m_duration;
    /// Time between two policy runs.
    Seconds m_period;
    Seconds m_elapsed{};
    Seconds m_wait{};
    std::vector<double> m_positions;
    std::vector<double> m_velocities;
};
