// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file TrayMission.hpp
/// @brief One arm carries a tray of glasses from below, the other one drills
/// a path on it: the skills of each robot, the observers and bindings of
/// the temporal domain.
#pragma once

#include "Robotik/Robot/Robot.hpp"
#include "Robotik/Task/Mission.hpp"

#include <set>
#include <string>
#include <vector>

namespace dual_arm
{

// ****************************************************************************
/// @brief Mission of the tray scenario. Publishes @c dual_arm.overlap
/// (seconds during which both arms ran a skill) and @c dual_arm.contact
/// (seconds during which a tool touched one of the entities it works on).
// ****************************************************************************
class TrayMission final: public robotik::Mission
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

    void addCarrierSkills(robotik::Cell& p_cell,
                          robotik::Robot const& p_robot);
    void addDrillSkills(robotik::Cell& p_cell,
                        robotik::Robot const& p_robot);
    void addPlanning(robotik::Cell& p_cell);

private:

    std::vector<std::string> m_trays;
    std::vector<std::string> m_pads;
    /// Trays the drill followed its path on (its skill succeeded).
    std::set<std::string> m_drilled;
    /// "touching(tool, workpiece)" of each tool and what it works on.
    std::vector<std::string> m_touches;
    double m_overlap = 0.0;
    double m_contact = 0.0;
};

} // namespace dual_arm
