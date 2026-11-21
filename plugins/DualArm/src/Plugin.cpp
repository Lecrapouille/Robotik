// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

// Two arms in one cell, planned by PDDL. The same mission runs the three
// conveyor scenarios: only the planner settings differ. The tray scenario
// plans with durative actions.

#include "ConveyorMission.hpp"
#include "TrayMission.hpp"

#include "Robotik/Package/Plugin/PluginAPI.hpp"

/// Loads the mission chosen by the scenario.
class DualArmPlugin final: public robotik::MissionPlugin
{
public:

    static robotik::PluginInfo describe()
    {
        return { .id = "robotik.dual_arm" };
    }

    DualArmPlugin()
        : MissionPlugin(
              { { "conveyor", robotik::mission<dual_arm::ConveyorMission>() },
                { "conveyor_sequential", robotik::mission<dual_arm::ConveyorMission>() },
                { "conveyor_pabt", robotik::mission<dual_arm::ConveyorMission>() },
                { "tray", robotik::mission<dual_arm::TrayMission>() } })
    {
    }
};

ROBOTIK_EXPORT_PLUGIN(DualArmPlugin)
