// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "WaveLogic.hpp"

#include "RobotIK/Task/RoboticCell.hpp"

#include <algorithm>
#include <cmath>

void WaveLogic::reset(robotik::RoboticCell& p_cell, robotik::Seed /*p_seed*/)
{
    // Find the joint once: act() runs at each step and uses the handle.
    // A typed handle: its angle is in radians, the compiler checks it.
    m_base = p_cell.robot().getJoints().requireRevolute("joint1");
    m_swing = 0.0;
    m_tool_y = 0.0;
}

void WaveLogic::act(robotik::RobotContext& p_context, robotik::Seconds /*p_dt*/)
{
    robotik::Robot& robot = p_context.robot;
    robotik::JointSet& joints = robot.getJoints();

    // Command: a target angle. The joint controller moves toward it.
    robotik::Radians const home = joints.home(m_base);
    robotik::Radians const offset(0.6 * std::sin(p_context.time.value()));
    joints.moveTo(m_base, home + offset);

    // Measure: the angle reached, and where the tool is (forward kinematics,
    // in the robot base frame).
    m_swing =
        std::max(m_swing, std::abs((joints.position(m_base) - home).value()));
    m_tool_y = robot.requireFramePose(robot.getTool()).position.y;
}

void WaveLogic::measure(robotik::RoboticCell const& /*p_cell*/,
                        robotik::Metrics& p_metrics) const
{
    p_metrics.set("wave.swing", m_swing);
    p_metrics.set("wave.tool_y", m_tool_y);
}

robotik::Status WaveLogic::status(robotik::RobotContext const& p_context) const
{
    if (p_context.time >= robotik::Seconds(6.0))
    {
        return robotik::Status::SUCCESS;
    }
    return robotik::Status::RUNNING;
}
