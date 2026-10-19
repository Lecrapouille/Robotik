// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "PainterMission.hpp"

#include "Robotik/Plugin/PluginAPI.hpp"

#include <memory>
#include <string_view>

/// Loads the painter mission.
class PainterPlugin final: public robotik::Plugin
{
public:

    static robotik::PluginInfo describe()
    {
        return { .id = "robotik.painter" };
    }

    RobotikPluginStatus setup(robotik::Host& p_host,
                              std::string_view p_scenario) override
    {
        m_mission.reset();
        if (p_scenario != "painter" && p_scenario != "painter_pddl")
        {
            p_host.error("unknown painter scenario");
            return ROBOTIK_PLUGIN_ERR_SETUP;
        }
        m_mission =
            std::make_unique<PainterMission>(p_scenario == "painter_pddl");
        p_host.bindMission(*m_mission);
        return ROBOTIK_PLUGIN_OK;
    }

    RobotikPluginStatus shutdown(robotik::Host& /*p_host*/) override
    {
        m_mission.reset();
        return ROBOTIK_PLUGIN_OK;
    }

private:

    std::unique_ptr<PainterMission> m_mission;
};

ROBOTIK_EXPORT_PLUGIN(PainterPlugin)
