// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file Conveyor.hpp
/// @brief A belt of the cell: the skill commands it, the mission moves the
/// part it carries (abstract physics: the part slides at a fixed speed).
#pragma once

#include "Robotik/Math/Geometry.hpp"
#include "Robotik/Skills/Skill.hpp"

#include <string>

namespace robotik
{
class Simulation;
}

namespace dual_arm
{

// ****************************************************************************
/// @brief The belt: carries one part at a time to a point of the world.
// ****************************************************************************
class Conveyor
{
public:

    /// Resource of the belt: the skill holds it alone, a fault stops it.
    static constexpr char const* RESOURCE = "conveyor";

    /// @brief Starts carrying @p_part to @p_goal (world frame, x and y).
    void carry(std::string p_part, robotik::Vector3 const& p_goal);

    /// @brief Stops the belt where it is.
    void stop();

    /// @brief Forgets the last delivery (new episode).
    void reset();

    /// @brief Part brought to its goal by the last @ref carry, or empty.
    [[nodiscard]] std::string const& delivered() const
    {
        return m_delivered;
    }

    // -------------------------------------------------------------------------
    /// @brief Moves the carried part by @p_dt, unless the resource failed.
    // -------------------------------------------------------------------------
    void step(robotik::Simulation& p_simulation, Seconds p_dt);

private:

    std::string m_part;
    std::string m_delivered;
    robotik::Vector3 m_goal = robotik::zero3();
    bool m_running = false;
};

// ****************************************************************************
/// @brief Convey(part, zone): the belt brings the part to the believed
/// centre of the zone.
// ****************************************************************************
class ConveySkill final: public robotik::Skill
{
public:

    ConveySkill(Conveyor& p_conveyor, std::string p_part, std::string p_zone);

    void reset() override
    {
        m_started = false;
        m_why.clear();
    }

    robotik::Status tick(robotik::RobotContext& p_context, Seconds p_dt) override;
    void cancel(robotik::RobotContext& p_context) override;

    [[nodiscard]] std::string why() const override
    {
        return m_why;
    }

private:

    Conveyor& m_conveyor;
    std::string m_part;
    std::string m_zone;
    bool m_started = false;
    std::string m_why;
};

} // namespace dual_arm
