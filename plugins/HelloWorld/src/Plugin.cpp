// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "HelloMission.hpp"
#include "ReachMission.hpp"
#include "WaveMission.hpp"
#include "WaveSkill.hpp"

#include "Robotik/Package/Plugin/PluginAPI.hpp"
#include "Robotik/Package/Simulation.hpp"

#include <string>

// ****************************************************************************
/// The plugin: one mission per scenario of plugin.yaml. The menu item, the
/// panel and the key binding below are optional.
// ****************************************************************************
class HelloWorldPlugin final: public robotik::MissionPlugin
{
public:

    /// The id must match plugin.id in plugin.yaml.
    static robotik::PluginInfo describe()
    {
        return { .id = "robotik.hello_world" };
    }

    /// Scenario id (as in plugin.yaml) -> the mission it runs.
    HelloWorldPlugin()
        : MissionPlugin({ { "hello", robotik::mission<HelloMission>() },
                          { "wave", robotik::mission<WaveMission>() },
                          { "reach", robotik::mission<ReachMission>() },
                          { "skills", robotik::mission<SkillsMission>() } })
    {
    }

    /// Before the simulation exists. The base creates and binds the mission;
    /// then a key binding, a menu item and a panel.
    void setup(robotik::Host& p_host, std::string_view p_scenario) override
    {
        MissionPlugin::setup(p_host, p_scenario);
        m_host = p_host;
        m_updates = m_keys = m_homes = 0;
        p_host.subscribe(ROBOTIK_EVENT_KEY_PRESSED);
        p_host.addMenuItem("Hello world", "Home", [this] { home(); });
        p_host.addPanel("hello_world", "Hello world",
                        [this](robotik::Canvas const& p_canvas) { draw(p_canvas); });
    }

    /// After each simulation step.
    void update(robotik::Host& /*p_host*/, robotik::Seconds /*p_dt*/) override
    {
        ++m_updates;
    }

    /// Only the events subscribed in setup arrive here.
    void onEvent(robotik::Host& /*p_host*/, RobotikEvent const& p_event) override
    {
        if (p_event.kind == ROBOTIK_EVENT_KEY_PRESSED)
        {
            ++m_keys;
            if (p_event.code == 'H')
            {
                home();
            }
        }
    }

private:

    /// Asks the scheduler to run the generic Home skill.
    void home()
    {
        ++m_homes;
        if (robotik::Simulation* simulation = m_host.simulation())
        {
            robotik::SkillScheduler& skills = simulation->cell().skills();
            if (robotik::SkillId const id = skills.find("Home"); id != robotik::NO_SKILL)
            {
                skills.request(id);
            }
        }
    }

    /// Draws the counters of the plugin, and the reach error when it runs.
    void draw(robotik::Canvas const& p_canvas) const
    {
        p_canvas.text("scenario " + scenario());
        p_canvas.text("updates=" + std::to_string(m_updates) +
                      " keys=" + std::to_string(m_keys) +
                      " homes=" + std::to_string(m_homes));
        if (ReachMission const* reach = mission<ReachMission>())
        {
            p_canvas.text("reach error " + std::to_string(reach->error()) + " m");
        }
    }

    /// Kept to reach the simulation from the menu.
    robotik::Host m_host;
    int m_updates = 0;
    int m_keys = 0;
    int m_homes = 0;
};

ROBOTIK_EXPORT_PLUGIN(HelloWorldPlugin)
