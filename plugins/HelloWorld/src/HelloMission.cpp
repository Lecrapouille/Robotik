// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "HelloMission.hpp"

#include "Robotik/Task/Cell.hpp"

void HelloMission::reset(robotik::Cell& /*p_cell*/,
                         robotik::Seed /*p_seed*/)
{
    m_ticks = 0.0;
}

void HelloMission::step(robotik::Cell& /*p_cell*/, robotik::Seconds /*p_dt*/)
{
    m_ticks += 1.0;
}

void HelloMission::measure(robotik::Cell const& /*p_cell*/,
                           robotik::Metrics& p_metrics) const
{
    p_metrics.set("hello.ticks", m_ticks);
}

robotik::Status
HelloMission::status(robotik::Cell const& p_cell) const
{
    if (p_cell.time() >= robotik::Seconds(2.0))
    {
        return robotik::Status::SUCCESS;
    }
    return robotik::Status::RUNNING;
}
