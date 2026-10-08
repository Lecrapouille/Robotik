// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "FlyController.hpp"

#include <algorithm>
#include <cmath>

namespace
{

constexpr double MAX_SPEED_M_S = 1.7;
constexpr double SPEED_GAIN = 2.2;
constexpr double MAX_YAW_RATE = 1.15;
constexpr double LIFT_SPEED_M_S = 0.7;
constexpr double LIFT_GAIN = 2.0;
constexpr double GROUND_Z_M = 0.19;
constexpr double PI = 3.14159265358979323846;

//! @brief Clamps a command before it becomes a speed.
double clampUnit(double p_value, double p_low, double p_high)
{
    return std::clamp(p_value, p_low, p_high);
}

} // namespace

//! @brief Standing posture, wing stroke back to the start.
void FlyController::reset()
{
    m_posture = {};
    m_phase = 0.0;
}

//! @brief Integrates the thorax, then writes a posture the URDF can show.
//! Lift zero holds altitude: the weight is cancelled here, not in the URDF.
void FlyController::apply(FlyAction const& p_action,
                          FlyPlant& p_plant,
                          double p_dt)
{
    double const forward = clampUnit(p_action.forward, 0.0, 1.0);
    double const turn = clampUnit(p_action.turn, -1.0, 1.0);
    double const lift = clampUnit(p_action.lift, -1.0, 1.0);
    double const blend = std::min(1.0, SPEED_GAIN * p_dt);

    // Speed and climb ease toward the command. Yaw rate is applied at once.
    p_plant.speed += (forward * MAX_SPEED_M_S - p_plant.speed) * blend;
    p_plant.yaw_rate = turn * MAX_YAW_RATE;
    p_plant.yaw += p_plant.yaw_rate * p_dt;
    p_plant.x += p_plant.speed * std::cos(p_plant.yaw) * p_dt;
    p_plant.y += p_plant.speed * std::sin(p_plant.yaw) * p_dt;

    // Lift ease toward the command.
    double const lift_blend = std::min(1.0, LIFT_GAIN * p_dt);
    p_plant.vertical_speed +=
        (lift * LIFT_SPEED_M_S - p_plant.vertical_speed) * lift_blend;
    p_plant.z += p_plant.vertical_speed * p_dt;
    if (p_plant.z < GROUND_Z_M)
    {
        p_plant.z = GROUND_Z_M;
        p_plant.vertical_speed = std::max(0.0, p_plant.vertical_speed);
    }

    // x100 clock runs 10x slower than the animal, so 20 Hz here is 200 Hz.
    // sweep = 0.35 + 1.55*sin fills the URDF range [-1.2, 1.9].
    double const frequency = 8.0 + 12.0 * forward;
    m_phase += 2.0 * PI * frequency * p_dt;
    double const sweep = 0.35 + 1.55 * std::sin(m_phase);
    m_posture.wing_sweep = sweep;
    m_posture.wing_deviation = 0.12 * std::sin(2.0 * m_phase);
    m_posture.wing_pitch = 0.35 * std::cos(m_phase);
    m_posture.haltere = 0.75 * std::sin(m_phase);
    m_posture.neck_yaw = -0.35 * turn;
    m_posture.neck_pitch = 0.12;
    m_posture.abdomen_pitch = -0.25 * lift;

    // Legs stay down on the ground and tuck as the thorax climbs.
    double const airborne = clampUnit((p_plant.z - 0.28) / 0.45, 0.0, 1.0);
    m_posture.coxa_pitch = airborne * 0.45;
    m_posture.femur_pitch = airborne * -0.7;
    m_posture.tibia_pitch = airborne * 0.9;
    m_posture.tarsus_pitch = airborne * 0.35;
}
