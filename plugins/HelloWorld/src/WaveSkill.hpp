// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#pragma once

#include "Robotik/Runtime/Mission.hpp"
#include "Robotik/Skills/Skill.hpp"

// ****************************************************************************
//! @brief Lesson 4: a skill. It swings the base joint left, right, left,
//! right, then succeeds.
//!
//! A skill only sees the @ref robotik::RobotContext: the robot API and its
//! beliefs. It cannot tell a simulation from a real robot.
// ****************************************************************************
class WaveSkill final: public robotik::Skill
{
public:

    void reset() override;
    robotik::Status tick(robotik::RobotContext& p_context, Seconds p_dt) override;
    void cancel(robotik::RobotContext& p_context) override;

private:

    int m_waves = 0;
};

// ****************************************************************************
//! @brief Lesson 4: registers the skill "Wave". The behavior tree of the
//! scenario calls it by that name.
// ****************************************************************************
class SkillsMission final: public robotik::Mission
{
public:

    void setup(robotik::Simulation& p_simulation, robotik::SceneView* p_view) override;
};
