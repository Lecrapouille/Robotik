// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#pragma once

#include "Robotik/Common/Math.hpp"
#include "Robotik/Task/Mission.hpp"

#include <vector>

// ****************************************************************************
/// @brief Lesson 3: ask for a tool position, let inverse kinematics find the
/// joint angles.
///
/// At reset, the target is the current tool pose moved three centimetres
/// along x. Inverse kinematics gives one angle per joint; each step commands
/// them and publishes the remaining distance as "reach.error".
// ****************************************************************************
class ReachMission final: public robotik::Mission
{
public:

    void reset(robotik::Cell& p_cell,
               robotik::Seed p_seed) override;
    void step(robotik::Cell& p_cell, robotik::Seconds p_dt) override;
    void measure(robotik::Cell const& p_cell,
                 robotik::Metrics& p_metrics) const override;
    [[nodiscard]] robotik::Status
    status(robotik::Cell const& p_cell) const override;

    [[nodiscard]] double error() const
    {
        return m_error;
    }

private:

    robotik::Pose m_target{};
    std::vector<double> m_solution;
    double m_error = 1.0;
};
