// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "WalkMission.hpp"

#include "Robotik/Plugin/PluginAPI.hpp"
#include "Robotik/Runtime/Simulation.hpp"

#include <cmath>
#include <cstdio>
#include <memory>
#include <numbers>
#include <string>
#include <string_view>

/// Largest host time simulated in one update (s): a slow frame does not pile
/// up a lag the simulation never catches up.
#define MAX_LAG_S 0.05

/// Prints one or two numbers with a printf format.
static std::string format(char const* p_format, double p_a, double p_b = 0.0)
{
    char line[96];
    std::snprintf(line, sizeof(line), p_format, p_a, p_b);
    return line;
}

// ****************************************************************************
/// @brief Humanoid walking by a learned policy. The plugin owns the clock
/// (host.clock: plugin): it steps the simulation at the physics rate of the
/// training, whatever the host rate, so the policy runs at its own rate.
/// Its panel changes the walk command while the robot walks.
// ****************************************************************************
class HumanoidWalkPlugin final: public robotik::Plugin
{
public:

    static robotik::PluginInfo describe()
    {
        return { .id = "robotik.humanoid_walk" };
    }

    RobotikPluginStatus setup(robotik::Host& p_host,
                              std::string_view p_scenario) override
    {
        m_mission.reset();
        if (p_scenario != "walk" && p_scenario != "walk_circle")
        {
            p_host.error("unknown humanoid scenario");
            return ROBOTIK_PLUGIN_ERR_SETUP;
        }
        m_mission = std::make_unique<WalkMission>();
        p_host.bindMission(*m_mission);
        p_host.addPanel("humanoid", "Humanoid", &onDraw, this);
        return ROBOTIK_PLUGIN_OK;
    }

    RobotikPluginStatus update(robotik::Host& p_host, double p_dt) override
    {
        robotik::Simulation* simulation = p_host.simulation();
        if (m_mission == nullptr || simulation == nullptr)
        {
            return ROBOTIK_PLUGIN_OK;
        }
        robotik::Seconds const step = WalkMission::physicsStep();
        m_lag = std::min(m_lag + p_dt, MAX_LAG_S);
        // The physics goes on after the end of the run, so the simulator
        // shows what follows a failure (a robot without IMU falls). The
        // verdict is already frozen; the headless host stops at the end.
        // The epsilon absorbs rounding: 0.01 s is ten steps, not nine.
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
        static_cast<HumanoidWalkPlugin*>(p_user)->draw(
            robotik::Canvas(p_canvas));
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
        if (p_canvas.sliderInt("forward cm/s", vx, -50, 50))
        {
            command[0] = vx / 100.0;
        }
        if (p_canvas.sliderInt("left cm/s", vy, -30, 30))
        {
            command[1] = vy / 100.0;
        }
        if (p_canvas.sliderInt("turn deg/s", yaw, -28, 28))
        {
            command[2] = yaw / DEG;
        }
        if (p_canvas.button("Stop"))
        {
            command = {};
        }
        p_canvas.separator();
        p_canvas.text(format(
            "Path radius %.2f m (commanded %.2f m, exact after one turn)",
            m_mission->radius(),
            m_mission->commandedRadius()));
        p_canvas.text(format("Turns %.2f   distance %.1f m",
                             m_mission->turns(),
                             m_mission->distance()));
        if (m_mission->height() < 0.6)
        {
            p_canvas.textColored(0.90f, 0.25f, 0.25f, "The robot fell: Replay");
        }
        else
        {
            p_canvas.text(format("Pelvis height %.2f m", m_mission->height()));
        }
    }

    std::unique_ptr<WalkMission> m_mission;
    /// Host time not yet simulated (s).
    double m_lag = 0.0;
};

ROBOTIK_EXPORT_PLUGIN(HumanoidWalkPlugin)
