// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file ConveyorLogic.hpp
/// Two arms and a conveyor: the skills of each robot, the observers
/// and bindings of the PDDL domain, and the belt.
#pragma once

#include "Conveyor.hpp"

#include "Robotik/Robot/Robot.hpp"
#include "Robotik/Task/ScenarioLogic.hpp"

#include <string>
#include <vector>

namespace dual_arm
{

// ****************************************************************************
/// ScenarioLogic of the conveyor scenarios. Publishes @c dual_arm.overlap:
/// simulated seconds during which both arms ran a skill.
// ****************************************************************************
class ConveyorLogic final: public robotik::ScenarioLogic
{
public:

    void setup(robotik::RoboticCell& p_cell,
               robotik::SceneView* p_view) override;
    void reset(robotik::RoboticCell& p_cell, robotik::Seed p_seed) override;
    void simulate(robotik::RoboticCell& p_cell, robotik::Seconds p_dt) override;
    void observe(robotik::RoboticCell const& p_cell,
                 robotik::Seconds p_dt) override;
    void measure(robotik::RoboticCell const& p_cell,
                 robotik::Metrics& p_metrics) const override;

private:

    void addArmSkills(robotik::RoboticCell& p_cell,
                      robotik::Robot const& p_robot);
    void addPlanning(robotik::RoboticCell& p_cell);

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
