// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

// Two arms in one cell, planned by PDDL. The same scenario logic runs the three
// conveyor scenarios: only the planner settings differ. The tray scenario
// plans with durative actions.

#include "ConveyorLogic.hpp"
#include "TrayLogic.hpp"

#include "RobotIK/Package/Plugin/PluginAPI.hpp"

/// Loads the scenario logic chosen by the scenario.
class DualArmPlugin final: public robotik::ScenarioPlugin
{
public:

    static robotik::PluginInfo describe()
    {
        return { .id = "robotik.dual_arm" };
    }

    DualArmPlugin()
        : ScenarioPlugin(
              { { "conveyor", robotik::logic<dual_arm::ConveyorLogic>() },
                { "tray", robotik::logic<dual_arm::TrayLogic>() } })
    {
    }
};

ROBOTIK_EXPORT_PLUGIN(DualArmPlugin)
