// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "PickPlaceMission.hpp"

#include "Robotik/Plugin/PluginAPI.hpp"

#include <memory>
#include <string_view>

namespace
{

class PickPlacePlugin final: public robotik::Plugin
{
public:

    static robotik::PluginInfo describe()
    {
        return { "robotik.pick_and_place", "Pick and place BT", "1.0.0", "Manipulation" };
    }

    RobotikPluginStatus setup(robotik::Host& p_host, std::string_view p_scenario) override
    {
        m_mission.reset();
        if (p_scenario != "pick_and_place" && p_scenario != "pick_and_place_faults")
        {
            p_host.error("unknown pick-and-place scenario");
            return ROBOTIK_PLUGIN_ERR_SETUP;
        }
        m_mission = std::make_unique<PickPlaceMission>(true);
        p_host.bindMission(*m_mission);
        return ROBOTIK_PLUGIN_OK;
    }

    RobotikPluginStatus shutdown(robotik::Host& /*p_host*/) override
    {
        m_mission.reset();
        return ROBOTIK_PLUGIN_OK;
    }

private:

    std::unique_ptr<PickPlaceMission> m_mission;
};

} // namespace

ROBOTIK_EXPORT_PLUGIN(PickPlacePlugin)
