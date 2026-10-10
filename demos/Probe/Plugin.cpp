// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "Robotik/Plugin/PluginAPI.hpp"

#include <string>

namespace
{

class ProbePlugin final: public robotik::Plugin
{
public:

    static robotik::PluginInfo describe()
    {
        return { "robotik.probe", "Probe", "1.0.0", "Test" };
    }

    RobotikPluginStatus setup(robotik::Host& p_host, std::string_view p_scenario) override
    {
        if (p_scenario == "broken")
        {
            p_host.error("broken scenario");
            return ROBOTIK_PLUGIN_ERR_SETUP;
        }
        m_scenario = std::string(p_scenario);
        m_updates = 0;
        m_keys = 0;
        m_homes = 0;
        p_host.subscribe(ROBOTIK_EVENT_KEY_PRESSED);
        p_host.addMenuItem("Probe", "Home", &onHome, this);
        p_host.addPanel("probe", "Probe", &onDraw, this);
        return ROBOTIK_PLUGIN_OK;
    }

    RobotikPluginStatus update(robotik::Host& /*p_host*/, double /*p_dt*/) override
    {
        ++m_updates;
        return ROBOTIK_PLUGIN_OK;
    }

    RobotikPluginStatus onEvent(robotik::Host& /*p_host*/,
                                RobotikEvent const& p_event) override
    {
        if (p_event.kind == ROBOTIK_EVENT_KEY_PRESSED)
        {
            ++m_keys;
        }
        return ROBOTIK_PLUGIN_OK;
    }

    RobotikPluginStatus shutdown(robotik::Host& /*p_host*/) override
    {
        m_scenario.clear();
        return ROBOTIK_PLUGIN_OK;
    }

    void home()
    {
        ++m_homes;
    }

    void draw(robotik::Canvas const& p_canvas) const
    {
        p_canvas.text(m_scenario + " updates=" + std::to_string(m_updates) +
                      " keys=" + std::to_string(m_keys) +
                      " homes=" + std::to_string(m_homes));
    }

private:

    static void onHome(void* p_user)
    {
        static_cast<ProbePlugin*>(p_user)->home();
    }

    static void onDraw(void* p_user, RobotikCanvas const* p_canvas)
    {
        static_cast<ProbePlugin*>(p_user)->draw(robotik::Canvas(p_canvas));
    }

    std::string m_scenario;
    int m_updates = 0;
    int m_keys = 0;
    int m_homes = 0;
};

} // namespace

ROBOTIK_EXPORT_PLUGIN(ProbePlugin)
