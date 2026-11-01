// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file Pusher.hpp
/// The sorter of the conveyor cell: a pusher takes one part off the infeed
/// table and across to the lane of its pick zone; the lane is a conveyor
/// belt of the cell (a core device) that carries it to the end stop. Only
/// the pusher moves the part itself (abstract physics: the part slides at
/// its speed).
#pragma once

#include "Robotik/Common/Math.hpp"
#include "Robotik/Task/Skills/Skill.hpp"

#include <optional>
#include <string>
#include <vector>

namespace robotik
{
class Cell;
class ConveyorBelt;
} // namespace robotik

namespace dual_arm
{

// ****************************************************************************
/// The pusher: moves one part at a time along a path of the world (off the
/// infeed table, across to a lane).
// ****************************************************************************
class Pusher
{
public:

    /// Resource of the pusher: the skill holds it alone, a fault stops it.
    static constexpr char const* RESOURCE = "pusher";

    /// Starts pushing @p_part through @p_path (world frame, x and y).
    void carry(std::string p_part, std::vector<robotik::Vector3> p_path);

    /// Stops the pusher where it is.
    void stop();

    /// Forgets the last push (new episode).
    void reset();

    /// Part brought to the end of its path by the last @ref carry, or empty.
    [[nodiscard]] std::string const& pushed() const
    {
        return m_pushed;
    }

    // -------------------------------------------------------------------------
    /// Moves the pushed part by @p_dt, unless the resource failed.
    // -------------------------------------------------------------------------
    void step(robotik::Cell& p_cell, robotik::Seconds p_dt);

private:

    std::string m_part;
    std::string m_pushed;
    /// Points left to reach, the next one first.
    std::vector<robotik::Vector3> m_path;
    bool m_running = false;
};

// ****************************************************************************
/// Convey(part, feeder, zone): the pusher takes the part off the feeder and
/// across to the lane of the zone; the belt of the lane carries it to the end
/// stop, on the zone. Done when the believed part rests on the zone.
// ****************************************************************************
class ConveySkill final: public robotik::Skill
{
public:

    ConveySkill(Pusher& p_pusher,
                robotik::ConveyorBelt& p_lane,
                std::string p_part,
                std::string p_feeder,
                std::string p_zone);

    void reset() override
    {
        m_started = false;
        m_last.reset();
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

    Pusher& m_pusher;
    robotik::ConveyorBelt& m_lane;
    std::string m_part;
    std::string m_feeder;
    std::string m_zone;
    bool m_started = false;
    /// Believed position of the part at the last tick, once on its lane.
    std::optional<robotik::Vector3> m_last;
    std::string m_why;
};

} // namespace dual_arm
