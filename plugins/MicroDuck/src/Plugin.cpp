// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "DuckMission.hpp"

#include "Robotik/Plugin/PluginAPI.hpp"
#include "Robotik/Runtime/Simulation.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <memory>
#include <numbers>
#include <string>
#include <string_view>

/// Largest host time simulated in one update (s): a slow frame does not pile
/// up a lag the simulation never catches up.
#define MAX_LAG_S 0.05
/// Trunk below this stands for a duck that is down (home is about 0.12 m).
#define FALLEN_HEIGHT_M 0.07

/// Prints one number with a printf format.
static std::string format(char const* p_format, double p_value)
{
    char line[96];
    std::snprintf(line, sizeof(line), p_format, p_value);
    return line;
}

// ****************************************************************************
/// @brief Microduck walking by a learned policy. The plugin owns the clock
/// (host.clock: plugin): it steps the simulation at the physics rate of the
/// training, whatever the host rate, so the policy runs at 50 Hz.
/// Its panel changes the walk command while the duck walks.
// ****************************************************************************
class MicroDuckPlugin final: public robotik::Plugin
{
public:

    static robotik::PluginInfo describe()
    {
        return { .id = "robotik.microduck" };
    }

    RobotikPluginStatus setup(robotik::Host& p_host,
                              std::string_view p_scenario) override
    {
        m_mission.reset();
        if (p_scenario != "drive" && p_scenario != "tour" &&
            p_scenario != "walk_forward")
        {
            p_host.error("unknown microduck scenario");
            return ROBOTIK_PLUGIN_ERR_SETUP;
        }
        m_mission = std::make_unique<DuckMission>();
        p_host.bindMission(*m_mission);
        p_host.addPanel("microduck", "Micro Duck", &onDraw, this);
        return ROBOTIK_PLUGIN_OK;
    }

    RobotikPluginStatus update(robotik::Host& p_host, double p_dt) override
    {
        robotik::Simulation* simulation = p_host.simulation();
        if (m_mission == nullptr || simulation == nullptr)
        {
            return ROBOTIK_PLUGIN_OK;
        }
        robotik::Seconds const step = m_mission->physicsStep();
        m_lag = std::min(m_lag + p_dt, MAX_LAG_S);
        // The physics goes on after the end of the run, so the simulator
        // shows a fall. The verdict is already frozen; the headless host
        // stops at the end.
        while (m_lag + 1e-9 >= step.value())
        {
            simulation->step(step);
            m_lag -= step.value();
        }
        return ROBOTIK_PLUGIN_OK;
    }

    RobotikPluginStatus shutdown(robotik::Host& /*p_host*/) override
    {
        m_mission.reset();
        return ROBOTIK_PLUGIN_OK;
    }

private:

    static void onDraw(void* p_user, RobotikCanvas const* p_canvas)
    {
        static_cast<MicroDuckPlugin*>(p_user)->draw(robotik::Canvas(p_canvas));
    }

    void draw(robotik::Canvas const& p_canvas)
    {
        if (m_mission == nullptr)
        {
            return;
        }
        // Sliders speak cm/s and deg/s; the command is SI.
        std::array<double, 3>& command = m_mission->command();
        constexpr double DEG = 180.0 / std::numbers::pi;
        int vx = static_cast<int>(std::lround(command[0] * 100.0));
        int vy = static_cast<int>(std::lround(command[1] * 100.0));
        int yaw = static_cast<int>(std::lround(command[2] * DEG));
        p_canvas.text("Walk command (applied at once)");
        if (p_canvas.sliderInt("forward cm/s", vx, -40, 40))
        {
            command[0] = vx / 100.0;
        }
        if (p_canvas.sliderInt("left cm/s", vy, -30, 30))
        {
            command[1] = vy / 100.0;
        }
        if (p_canvas.sliderInt("turn deg/s", yaw, -57, 57))
        {
            command[2] = yaw / DEG;
        }
        if (p_canvas.button("Stop"))
        {
            command = {};
        }
        p_canvas.separator();
        p_canvas.text(format("Distance %.2f m", m_mission->distance()));
        if (m_mission->height() < FALLEN_HEIGHT_M)
        {
            p_canvas.textColored(0.90f, 0.25f, 0.25f, "The duck fell: Replay");
        }
        else
        {
            p_canvas.text(format("Trunk height %.3f m", m_mission->height()));
        }
    }

    std::unique_ptr<DuckMission> m_mission;
    /// Host time not yet simulated (s).
    double m_lag = 0.0;
};

ROBOTIK_EXPORT_PLUGIN(MicroDuckPlugin)
