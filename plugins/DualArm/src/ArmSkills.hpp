// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file ArmSkills.hpp
/// @brief Skills of one arm with a vacuum cup, in a cell of several robots.
///
/// The beliefs are shared by the robots, in the world frame; each skill
/// brings them into the base frame of its own robot (@ref inBase).
#pragma once

#include "Robotik/Task/Skills/MotionSkills.hpp"

#include <string>

namespace robotik
{
class Robot;
class VacuumGripper;
} // namespace robotik

namespace dual_arm
{

// ----------------------------------------------------------------------------
/// @brief A point of the beliefs in the base frame of the robot of
/// @p_context: beliefs are in the world frame when the robots share them.
// ----------------------------------------------------------------------------
[[nodiscard]] robotik::Vector3 inBase(robotik::RobotContext const& p_context,
                                      robotik::Vector3 const& p_belief);

// ----------------------------------------------------------------------------
/// @brief Tool pointing down above the point @p_at of the base, turned so
/// that the last joint stays near 0 (far from its limits): a cup or a
/// pointer does not care about its turn around its axis.
// ----------------------------------------------------------------------------
[[nodiscard]] robotik::Quaternion pointingDown(robotik::Vector3 const& p_at);

/// @brief Robot @p_id of the cell of @p_context, or null.
[[nodiscard]] robotik::Robot const*
teammate(robotik::RobotContext const& p_context, std::string_view p_id);

/// @brief First vacuum cup of @p_robot, or null.
[[nodiscard]] robotik::VacuumGripper* cupOf(robotik::Robot const& p_robot);

/// @brief Id of the object the cup of @p_robot holds, as its vacuum sensor
/// reports it, or empty.
[[nodiscard]] std::string heldObject(robotik::Robot const& p_robot);

// ****************************************************************************
/// @brief Brings the cup @p_clearance above the top of an object (a part,
/// or the walls of a bin), pointing down.
// ****************************************************************************
class HoverSkill final: public robotik::Skill
{
public:

    HoverSkill(std::string p_object, robotik::Length p_clearance);

    void reset() override;
    robotik::Status tick(robotik::RobotContext& p_context,
                         robotik::Seconds p_dt) override;
    void cancel(robotik::RobotContext& p_context) override;

    [[nodiscard]] std::string why() const override
    {
        return m_why;
    }

private:

    std::string m_object;
    robotik::Length m_clearance;
    robotik::MoveTCPSkill m_move;
    bool m_planned = false;
    std::string m_why;
};

// ****************************************************************************
/// @brief Turns the suction on and waits until @p_object is held.
// ****************************************************************************
class GraspSkill final: public robotik::Skill
{
public:

    explicit GraspSkill(std::string p_object, robotik::Seconds p_timeout = robotik::Seconds(0.5));

    void reset() override
    {
        m_waited = robotik::Seconds{};
        m_why.clear();
    }

    robotik::Status tick(robotik::RobotContext& p_context,
                         robotik::Seconds p_dt) override;
    void cancel(robotik::RobotContext& p_context) override;

    [[nodiscard]] std::string why() const override
    {
        return m_why;
    }

private:

    std::string m_object;
    robotik::Seconds m_timeout;
    robotik::Seconds m_waited{};
    std::string m_why;
};

// ****************************************************************************
/// @brief Turns the suction off and waits until nothing is held.
// ****************************************************************************
class ReleaseSkill final: public robotik::Skill
{
public:

    explicit ReleaseSkill(robotik::Seconds p_timeout = robotik::Seconds(0.5))
        : m_timeout(p_timeout)
    {
    }

    void reset() override
    {
        m_waited = robotik::Seconds{};
        m_why.clear();
    }

    robotik::Status tick(robotik::RobotContext& p_context,
                         robotik::Seconds p_dt) override;

    [[nodiscard]] std::string why() const override
    {
        return m_why;
    }

private:

    robotik::Seconds m_timeout;
    robotik::Seconds m_waited{};
    std::string m_why;
};

} // namespace dual_arm
