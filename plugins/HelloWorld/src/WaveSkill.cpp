// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "WaveSkill.hpp"

#include "Robotik/Robot/Robot.hpp"
#include "Robotik/Task/RobotContext.hpp"
#include "Robotik/Task/Cell.hpp"

void WaveSkill::reset()
{
    m_waves = 0;
}

robotik::Status WaveSkill::tick(robotik::RobotContext& p_context,
                                robotik::Seconds /*p_dt*/)
{
    robotik::JointSet& joints = p_context.robot.joints();
    robotik::JointId const base = joints.find("joint1");
    if (base == robotik::NO_JOINT)
    {
        return robotik::Status::FAILURE;
    }

    double const target = (m_waves % 2 == 0) ? 0.5 : -0.5;
    joints.moveTo(base, target);
    if (joints.reached(base, 0.02))
    {
        ++m_waves;
    }
    return m_waves >= 4 ? robotik::Status::SUCCESS : robotik::Status::RUNNING;
}

void WaveSkill::cancel(robotik::RobotContext& p_context)
{
    // Cancelled or preempted: stop where we are.
    p_context.robot.joints().hold();
}

void SkillsMission::setup(robotik::Cell& p_cell,
                          robotik::SceneView* /*p_view*/)
{
    // The skill reserves the actuator "arm" while it runs: no other skill
    // can move the arm at the same time.
    robotik::SkillDescription wave;
    wave.name = "Wave";
    wave.resources = { p_cell.robot().resources().require("arm") };
    wave.priority = 100;
    p_cell.skills().add<WaveSkill>(std::move(wave));
}
