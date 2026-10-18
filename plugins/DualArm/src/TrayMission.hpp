// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file TrayMission.hpp
/// @brief One arm carries a tray of glasses, the other one visits them: the
/// skills of each robot, the observers and bindings of the temporal domain.
#pragma once

#include "Robotik/Robot/Robot.hpp"
#include "Robotik/Runtime/Mission.hpp"

#include <set>
#include <string>
#include <vector>

namespace dual_arm
{

// ****************************************************************************
/// @brief Mission of the tray scenario. Publishes @c dual_arm.overlap
/// (seconds during which both arms ran a skill) and @c dual_arm.visited
/// (glasses a tool passed above).
// ****************************************************************************
class TrayMission final: public robotik::Mission
{
public:

    void setup(robotik::Simulation& p_simulation, robotik::SceneView* p_view) override;
    void reset(robotik::Simulation& p_simulation, robotik::Seed p_seed) override;
    void step(robotik::Simulation& p_simulation, Seconds p_dt) override;
    void measure(robotik::Simulation const& p_simulation, robotik::Metrics& p_metrics) const override;

private:

    void addCarrierSkills(robotik::Simulation& p_simulation, robotik::Robot const& p_robot);
    void addVisitorSkills(robotik::Simulation& p_simulation, robotik::Robot const& p_robot);
    void addPlanning(robotik::Simulation& p_simulation);

private:

    std::vector<std::string> m_glasses;
    std::vector<std::string> m_trays;
    std::vector<std::string> m_pads;
    /// Glasses a tool passed above, seen on the beliefs.
    std::set<std::string> m_visited;
    double m_overlap = 0.0;
};

} // namespace dual_arm
