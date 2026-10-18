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

#include "Robotik/Plugin/PluginAPI.hpp"
#include "Robotik/Runtime/Simulation.hpp"

#include <memory>
#include <string>

// ****************************************************************************
/// @brief The plugin: picks the mission of the chosen scenario, adds a menu
/// item, a panel and a key binding. Everything else is optional.
// ****************************************************************************
class HelloWorldPlugin final: public robotik::Plugin
{
public:

    /// The id must match plugin.id in plugin.yaml.
    static robotik::PluginInfo describe()
    {
        return { .id = "robotik.hello_world" };
    }

    /// Called before the simulation exists: create the mission and hand it
    /// to the host, which passes it to the simulation.
    RobotikPluginStatus setup(robotik::Host& p_host, std::string_view p_scenario) override
    {
        m_scenario = std::string(p_scenario);
        m_updates = 0;
        m_keys = 0;
        m_homes = 0;
        m_reach = nullptr;

        if (p_scenario == "hello")
        {
            m_mission = std::make_unique<HelloMission>();
        }
        else if (p_scenario == "wave")
        {
            m_mission = std::make_unique<WaveMission>();
        }
        else if (p_scenario == "reach")
        {
            auto reach = std::make_unique<ReachMission>();
            m_reach = reach.get();
            m_mission = std::move(reach);
        }
        else if (p_scenario == "skills")
        {
            m_mission = std::make_unique<SkillsMission>();
        }
        else
        {
            p_host.error("unknown hello world scenario: " + m_scenario);
            return ROBOTIK_PLUGIN_ERR_SETUP;
        }

        p_host.bindMission(*m_mission);
        p_host.subscribe(ROBOTIK_EVENT_KEY_PRESSED);
        p_host.addMenuItem("Hello world", "Home", &onHome, this);
        p_host.addPanel("hello_world", "Hello world", &onDraw, this);
        m_host = p_host;
        return ROBOTIK_PLUGIN_OK;
    }

    /// Called after each simulation step.
    RobotikPluginStatus update(robotik::Host& /*p_host*/, double /*p_dt*/) override
    {
        ++m_updates;
        return ROBOTIK_PLUGIN_OK;
    }

    /// Only the events subscribed in setup arrive here.
    RobotikPluginStatus onEvent(robotik::Host& /*p_host*/,
                                RobotikEvent const& p_event) override
    {
        if (p_event.kind == ROBOTIK_EVENT_KEY_PRESSED)
        {
            ++m_keys;
            if (p_event.code == 'H')
            {
                home();
            }
        }
        return ROBOTIK_PLUGIN_OK;
    }

    /// The host already destroyed the simulation: free the mission.
    RobotikPluginStatus shutdown(robotik::Host& /*p_host*/) override
    {
        m_reach = nullptr;
        m_mission.reset();
        return ROBOTIK_PLUGIN_OK;
    }

    /// Requests the built-in skill "Home". Missions that command the joints
    /// at every step (wave, reach) overwrite it at the next step.
    void home()
    {
        ++m_homes;
        robotik::Simulation* simulation = m_host.simulation();
        if (simulation == nullptr)
        {
            return;
        }
        robotik::SkillId const id = simulation->skills().find("Home");
        if (id != robotik::NO_SKILL)
        {
            simulation->skills().request(id);
        }
    }

    void draw(robotik::Canvas const& p_canvas) const
    {
        p_canvas.text("scenario " + m_scenario);
        p_canvas.text("updates=" + std::to_string(m_updates) +
                      " keys=" + std::to_string(m_keys) +
                      " homes=" + std::to_string(m_homes));
        if (m_reach != nullptr)
        {
            p_canvas.text("reach error " + std::to_string(m_reach->error()) + " m");
        }
    }

private:

    // The host calls plain C functions: forward to the instance.
    static void onHome(void* p_user)
    {
        static_cast<HelloWorldPlugin*>(p_user)->home();
    }

    static void onDraw(void* p_user, RobotikCanvas const* p_canvas)
    {
        static_cast<HelloWorldPlugin*>(p_user)->draw(robotik::Canvas(p_canvas));
    }

    robotik::Host m_host;
    std::string m_scenario;
    std::unique_ptr<robotik::Mission> m_mission;
    ReachMission const* m_reach = nullptr;
    int m_updates = 0;
    int m_keys = 0;
    int m_homes = 0;
};

ROBOTIK_EXPORT_PLUGIN(HelloWorldPlugin)
