// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#pragma once

#include "Robotik/Task/Mission.hpp"

// ****************************************************************************
/// Lesson 2: command a joint, read the tool pose.
///
/// The base joint follows a sine around its home angle. Each step reads the
/// pose of the tool frame (forward kinematics) and publishes how far the
/// joint swung. The mission succeeds after six seconds.
// ****************************************************************************
class WaveMission final: public robotik::Mission
{
public:

    void reset(robotik::Cell& p_cell,
               robotik::Seed p_seed) override;
    void step(robotik::Cell& p_cell, robotik::Seconds p_dt) override;
    void measure(robotik::Cell const& p_cell,
                 robotik::Metrics& p_metrics) const override;
    [[nodiscard]] robotik::Status
    status(robotik::Cell const& p_cell) const override;

private:

    double m_swing = 0.0;
    double m_tool_y = 0.0;
};
