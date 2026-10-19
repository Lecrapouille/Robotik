// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#pragma once

#include "Robotik/Runtime/Mission.hpp"

// ****************************************************************************
/// @brief Lesson 1: the smallest mission.
///
/// It counts the simulation steps, publishes the count as the metric
/// "hello.ticks", and succeeds after two seconds. The scenario file asserts
/// on that metric.
// ****************************************************************************
class HelloMission final: public robotik::Mission
{
public:

    void reset(robotik::Simulation& p_simulation,
               robotik::Seed p_seed) override;
    void step(robotik::Simulation& p_simulation, robotik::Seconds p_dt) override;
    void measure(robotik::Simulation const& p_simulation,
                 robotik::Metrics& p_metrics) const override;
    [[nodiscard]] robotik::Status
    status(robotik::Simulation const& p_simulation) const override;

private:

    double m_ticks = 0.0;
};
