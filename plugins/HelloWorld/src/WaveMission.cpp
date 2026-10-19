// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "WaveMission.hpp"

#include "Robotik/Runtime/Simulation.hpp"

#include <algorithm>
#include <cmath>

void WaveMission::reset(robotik::Simulation& /*p_simulation*/,
                        robotik::Seed /*p_seed*/)
{
    m_swing = 0.0;
    m_tool_y = 0.0;
}

void WaveMission::step(robotik::Simulation& p_simulation, Seconds /*p_dt*/)
{
    robotik::Robot& robot = p_simulation.robot();
    robotik::JointSet& joints = robot.joints();

    // A typed handle: the angle is in radians, the compiler checks it.
    robotik::Revolute const base = joints.revolute("joint1");
    if (!base)
    {
        return;
    }

    // Command: a target angle. The joint controller moves toward it.
    Radians const home = joints.home(base);
    Radians const offset(0.6 * std::sin(p_simulation.time().value()));
    joints.moveTo(base, home + offset);

    // Measure: the angle reached, and where the tool is (forward kinematics,
    // in the robot base frame).
    m_swing =
        std::max(m_swing, std::abs((joints.position(base) - home).value()));
    m_tool_y = robot.framePose(robot.tool()).position.y;
}

void WaveMission::measure(robotik::Simulation const& /*p_simulation*/,
                          robotik::Metrics& p_metrics) const
{
    p_metrics.set("wave.swing", m_swing);
    p_metrics.set("wave.tool_y", m_tool_y);
}

robotik::Status
WaveMission::status(robotik::Simulation const& p_simulation) const
{
    if (p_simulation.time() >= Seconds(6.0))
    {
        return robotik::Status::SUCCESS;
    }
    return robotik::Status::RUNNING;
}
