// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "ManipulatorMission.hpp"

#include "Robotik/Robot/Robot.hpp"
#include "Robotik/Runtime/Simulation.hpp"

#include <algorithm>
#include <cmath>
#include <optional>

namespace
{

double clampJoint(robotik::JointSet const& p_joints,
                  robotik::JointId p_id,
                  double p_value)
{
    if (p_joints.isPrismatic(p_id))
    {
        robotik::PrismaticLimits const limits = p_joints.limits(p_joints.prismatic(p_id));
        if (limits.bounded())
        {
            return std::clamp(p_value, limits.lower.value(), limits.upper.value());
        }
    }
    else
    {
        robotik::RevoluteLimits const limits = p_joints.limits(p_joints.revolute(p_id));
        if (limits.bounded())
        {
            return std::clamp(p_value, limits.lower.value(), limits.upper.value());
        }
    }
    return p_value;
}

double separation(robotik::Pose const& p_from, robotik::Pose const& p_to)
{
    double const dx = p_from.position.x - p_to.position.x;
    double const dy = p_from.position.y - p_to.position.y;
    double const dz = p_from.position.z - p_to.position.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

void holdHome(robotik::Robot& p_robot)
{
    robotik::JointSet& joints = p_robot.joints();
    for (robotik::JointId id = 0; id < joints.size(); ++id)
    {
        joints.moveTo(id, joints.home(id));
    }
}

} // namespace

std::string_view ManipulatorMission::modeName() const
{
    switch (m_mode)
    {
        case Mode::Forward:
            return "Forward kinematics";
        case Mode::Inverse:
            return "Inverse kinematics";
        case Mode::Limits:
            return "Joint limits";
    }
    return "Manipulator";
}

void ManipulatorMission::reset(robotik::Simulation& p_simulation, robotik::Seed /*p_seed*/)
{
    m_home = false;
    m_have_target = false;
    m_solution.clear();
    m_samples = 0.0;
    m_error = 1.0;
    m_inside = 0.0;
    m_reached = 0.0;
    m_joint = robotik::NO_JOINT;

    robotik::Robot& robot = p_simulation.robot();
    robotik::JointSet& joints = robot.joints();
    if (m_mode == Mode::Inverse && !robot.tool().empty())
    {
        robotik::Pose const current = robot.framePose(robot.tool());
        for (double const offset : { 0.04, 0.02, -0.03 })
        {
            robotik::Pose target = current;
            target.position.x += offset;
            std::optional<std::vector<double>> const solution =
                robot.inverseKinematics(robot.tool(), target);
            if (solution)
            {
                m_target = target;
                m_solution = *solution;
                m_have_target = true;
                m_error = separation(current, m_target);
                break;
            }
        }
    }
    else if (m_mode == Mode::Limits)
    {
        m_joint = joints.find("joint2");
        if (m_joint != robotik::NO_JOINT)
        {
            robotik::RevoluteLimits const limits = joints.limits(joints.revolute(m_joint));
            m_inside = 1.0;
            joints.moveTo(m_joint, limits.bounded() ? limits.upper.value()
                                                    : joints.position(m_joint));
        }
    }
}

void ManipulatorMission::step(robotik::Simulation& p_simulation, Seconds /*p_dt*/)
{
    robotik::Robot& robot = p_simulation.robot();
    robotik::JointSet& joints = robot.joints();
    if (m_home)
    {
        holdHome(robot);
        m_home = false;
        return;
    }

    if (m_mode == Mode::Forward)
    {
        robotik::Revolute const base = joints.revolute("joint1");
        if (base)
        {
            double const home = joints.home(base).value();
            double const wave = 0.6 * std::sin(p_simulation.time().value());
            joints.moveTo(base, Radians(home + wave));
        }
        if (!robot.tool().empty())
        {
            (void)robot.framePose(robot.tool());
            m_samples += 1.0;
        }
        return;
    }

    if (m_mode == Mode::Inverse && m_have_target)
    {
        std::size_t const count = std::min(joints.size(), m_solution.size());
        for (robotik::JointId id = 0; id < count; ++id)
        {
            joints.moveTo(id, clampJoint(joints, id, m_solution[id]));
        }
        if (!robot.tool().empty())
        {
            m_error = separation(robot.framePose(robot.tool()), m_target);
        }
        return;
    }

    if (m_mode == Mode::Limits && m_joint != robotik::NO_JOINT)
    {
        robotik::RevoluteLimits const limits = joints.limits(joints.revolute(m_joint));
        double const upper = limits.bounded() ? limits.upper.value() : joints.position(m_joint);
        double const lower = limits.bounded() ? limits.lower.value() : upper;
        // The command is the limit itself, never past it.
        joints.moveTo(m_joint, upper);
        double const position = joints.position(m_joint);
        m_inside = (position <= upper + 1.0e-2 && position >= lower - 1.0e-2) ? 1.0 : 0.0;
        m_reached = std::abs(position - upper) < 0.05 ? 1.0 : 0.0;
    }
}

void ManipulatorMission::measure(robotik::Simulation const& /*p_simulation*/,
                                 robotik::Metrics& p_metrics) const
{
    p_metrics.set("fk.samples", m_samples);
    p_metrics.set("ik.error", m_error);
    p_metrics.set("limit.inside", m_inside);
    p_metrics.set("limit.reached", m_reached);
}

robotik::Status ManipulatorMission::status(robotik::Simulation const& p_simulation) const
{
    if (m_mode == Mode::Forward && p_simulation.time() >= Seconds(6.0))
    {
        return robotik::Status::SUCCESS;
    }
    if (m_mode == Mode::Inverse && m_have_target && m_error < 0.02)
    {
        return robotik::Status::SUCCESS;
    }
    if (m_mode == Mode::Limits && m_reached > 0.0)
    {
        return robotik::Status::SUCCESS;
    }
    if (m_mode == Mode::Inverse && !m_have_target && p_simulation.time() >= Seconds(1.0))
    {
        return robotik::Status::FAILURE;
    }
    return robotik::Status::RUNNING;
}
