// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "ReachMission.hpp"

#include "Robotik/Task/Cell.hpp"

#include <cmath>
#include <optional>

/// Distance between two poses.
static double distance(robotik::Pose const& p_from, robotik::Pose const& p_to)
{
    double const dx = p_from.position.x - p_to.position.x;
    double const dy = p_from.position.y - p_to.position.y;
    double const dz = p_from.position.z - p_to.position.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

void ReachMission::reset(robotik::Cell& p_cell,
                         robotik::Seed /*p_seed*/)
{
    robotik::Robot& robot = p_cell.robot();

    // The tool frame is "tcp" when a tool is mounted, "tool0" otherwise.
    robotik::Pose const start = robot.framePose(robot.tool());
    m_target = start;
    m_target.position.x += 0.03;

    // Inverse kinematics: no answer when the pose is out of reach.
    std::optional<std::vector<double>> const solution =
        robot.inverseKinematics(robot.tool(), m_target);
    m_solution = solution ? *solution : std::vector<double>{};
    m_error = distance(start, m_target);
}

void ReachMission::step(robotik::Cell& p_cell, robotik::Seconds /*p_dt*/)
{
    robotik::Robot& robot = p_cell.robot();
    robotik::JointSet& joints = robot.joints();
    for (robotik::JointId id = 0; id < joints.size() && id < m_solution.size();
         ++id)
    {
        joints.moveTo(id, m_solution[id]);
    }
    m_error = distance(robot.framePose(robot.tool()), m_target);
}

void ReachMission::measure(robotik::Cell const& /*p_cell*/,
                           robotik::Metrics& p_metrics) const
{
    p_metrics.set("reach.error", m_error);
}

robotik::Status
ReachMission::status(robotik::Cell const& /*p_cell*/) const
{
    if (m_solution.empty())
    {
        return robotik::Status::FAILURE;
    }
    return m_error < 0.005 ? robotik::Status::SUCCESS
                           : robotik::Status::RUNNING;
}
