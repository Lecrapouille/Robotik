// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#pragma once

#include "WalkPolicy.hpp"

#include "Robotik/Robot/Joints.hpp"
#include "Robotik/Task/Skills/Skill.hpp"

#include <array>
#include <string>
#include <vector>

// ****************************************************************************
/// @brief Joints of the policy, in its order, and the IMU that feeds it.
// ****************************************************************************
struct WalkBody
{
    std::vector<robotik::JointId> joints;
    std::string imu;
};

// ****************************************************************************
/// @brief Holds the default posture of the policy for a while: the robot
/// settles on its feet before walking.
// ****************************************************************************
class StandSkill final: public robotik::Skill
{
public:

    StandSkill(WalkBody p_body,
               std::vector<double> p_posture,
               robotik::Seconds p_duration);

    void reset() override;
    robotik::Status tick(robotik::RobotContext& p_context,
                         robotik::Seconds p_dt) override;
    void cancel(robotik::RobotContext& p_context) override;

private:

    WalkBody m_body;
    std::vector<double> m_posture;
    robotik::Seconds m_duration;
    robotik::Seconds m_elapsed{};
};

// ****************************************************************************
/// @brief Walks with a velocity command for a while (forever when the
/// duration is not positive). A forward speed and a yaw rate draw a circle of
/// radius vx / yaw rate. The command is read at each policy run: a panel may
/// change it while the robot walks.
///
/// The policy runs every decimation physics steps on the IMU and the joint
/// encoders; between two runs the joint PD loops hold its targets. The skill
/// fails when the IMU is lost or says the robot lies down; it then leaves the
/// joints in damping mode (velocity loop toward zero), as the LimX robot does.
// ****************************************************************************
class WalkSkill final: public robotik::Skill
{
public:

    WalkSkill(WalkBody p_body,
              WalkPolicy& p_policy,
              std::array<double, 3> const& p_command,
              robotik::Seconds p_duration,
              robotik::Seconds p_period);

    void reset() override;
    robotik::Status tick(robotik::RobotContext& p_context,
                         robotik::Seconds p_dt) override;
    void cancel(robotik::RobotContext& p_context) override;

private:

    WalkBody m_body;
    WalkPolicy& m_policy;
    /// Forward (m/s), lateral (m/s), yaw rate (rad/s), owned by the mission.
    std::array<double, 3> const& m_command;
    robotik::Seconds m_duration;
    /// Time between two policy runs.
    robotik::Seconds m_period;
    robotik::Seconds m_elapsed{};
    robotik::Seconds m_wait{};
    std::vector<double> m_positions;
    std::vector<double> m_velocities;
};
