// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#pragma once

#include "Robotik/Math/Random.hpp"
#include "Robotik/Runtime/Mission.hpp"

#include <array>
#include <cstdint>

inline constexpr std::size_t RL_OBSERVATIONS = 16;
inline constexpr std::size_t RL_ACTIONS = 4;

namespace robotik
{
class JointGroup;
class VacuumGripper;
} // namespace robotik

// ****************************************************************************
//! @brief One rendered pick-and-place episode driven by the scripted policy.
//!
//! @c step stays empty: the action itself calls @c Simulation::step, and a
//! policy step from inside that call would recurse.
// ****************************************************************************
class RlMission final: public robotik::Mission
{
public:

    void reset(robotik::Simulation& p_simulation, robotik::Seed p_seed) override;
    void step(robotik::Simulation& /*p_simulation*/, Seconds /*p_dt*/) override {}
    robotik::Status status(robotik::Simulation const& /*p_simulation*/) const override;

    //! @brief One policy action, or the pause before the next episode.
    void act(robotik::Simulation& p_simulation, double p_dt);

    bool converged = true;
    float mix = 1.0f;
    bool repeat = true;
    int max_steps = 120;
    std::array<float, RL_OBSERVATIONS> observation{};
    std::array<float, RL_ACTIONS> action{};
    std::uint32_t steps = 0;
    float episode_return = 0.0f;
    std::uint32_t delivered = 0;
    std::uint32_t failed = 0;
    bool done = false;
    bool success = false;
    bool ready = false;

private:

    robotik::Random m_noise{ robotik::Seed{ 1 } };
    robotik::JointGroup* m_arm = nullptr;
    robotik::VacuumGripper* m_gripper = nullptr;
    bool m_grasped = false;
    double m_pause = 0.0;
};
