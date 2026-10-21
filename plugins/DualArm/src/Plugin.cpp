// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

// Two arms in one cell, planned by PDDL. The same mission runs both
// conveyor scenarios: only planner.parallel differs. The tray scenario
// plans with durative actions.

#include "ConveyorMission.hpp"
#include "TrayMission.hpp"

#include "Robotik/Package/Plugin/PluginAPI.hpp"

#include <memory>
#include <string_view>

/// Loads the mission chosen by the scenario.
class DualArmPlugin final: public robotik::Plugin
{
public:

    static robotik::PluginInfo describe()
    {
        return { .id = "robotik.dual_arm" };
    }

    RobotikPluginStatus setup(robotik::Host& p_host,
                              std::string_view p_scenario) override
    {
        if (p_scenario == "conveyor" || p_scenario == "conveyor_sequential" ||
            p_scenario == "conveyor_grafcet")
        {
            m_mission = std::make_unique<dual_arm::ConveyorMission>();
            p_host.bindMission(*m_mission);
            return ROBOTIK_PLUGIN_OK;
        }
        if (p_scenario == "tray")
        {
            m_mission = std::make_unique<dual_arm::TrayMission>();
            p_host.bindMission(*m_mission);
            return ROBOTIK_PLUGIN_OK;
        }
        p_host.error("unknown dual arm scenario");
        return ROBOTIK_PLUGIN_ERR_SETUP;
    }

    RobotikPluginStatus shutdown(robotik::Host& /*p_host*/) override
    {
        m_mission.reset();
        return ROBOTIK_PLUGIN_OK;
    }

private:

    std::unique_ptr<robotik::Mission> m_mission;
};

ROBOTIK_EXPORT_PLUGIN(DualArmPlugin)
