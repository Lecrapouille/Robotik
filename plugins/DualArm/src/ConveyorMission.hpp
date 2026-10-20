// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file ConveyorMission.hpp
/// Two arms and a conveyor: the skills of each robot, the observers
/// and bindings of the PDDL domain, and the belt.
#pragma once

#include "Conveyor.hpp"

#include "Robotik/Robot/Robot.hpp"
#include "Robotik/Task/Mission.hpp"

#include <string>
#include <vector>

namespace dual_arm
{

// ****************************************************************************
/// Mission of the conveyor scenarios. Publishes @c dual_arm.overlap:
/// simulated seconds during which both arms ran a skill.
// ****************************************************************************
class ConveyorMission final: public robotik::Mission
{
public:

    void setup(robotik::Cell& p_cell,
               robotik::SceneView* p_view) override;
    void reset(robotik::Cell& p_cell,
               robotik::Seed p_seed) override;
    void step(robotik::Cell& p_cell, robotik::Seconds p_dt) override;
    void measure(robotik::Cell const& p_cell,
                 robotik::Metrics& p_metrics) const override;

private:

    void addArmSkills(robotik::Cell& p_cell,
                      robotik::Robot const& p_robot);
    void addPlanning(robotik::Cell& p_cell);

private:

    Conveyor m_conveyor;
    /// World entities of each kind (ids).
    std::vector<std::string> m_parts;
    std::vector<std::string> m_feeders;
    std::vector<std::string> m_zones;
    std::vector<std::string> m_bins;
    double m_overlap = 0.0;
};

} // namespace dual_arm
