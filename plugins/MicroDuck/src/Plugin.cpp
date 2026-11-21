// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "DuckMission.hpp"

#include "Robotik/Package/Plugin/PluginAPI.hpp"
#include "Robotik/Package/Simulation.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <numbers>
#include <string>

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
/// Microduck walking by a learned policy. The plugin owns the clock
/// (host.clock: plugin): it steps the simulation at the physics rate of the
/// training, whatever the host rate, so the policy runs at 50 Hz.
/// Its panel changes the walk command while the duck walks.
// ****************************************************************************
class MicroDuckPlugin final: public robotik::MissionPlugin
{
public:

    static robotik::PluginInfo describe()
    {
        return { .id = "robotik.microduck" };
    }

    MicroDuckPlugin()
        : MissionPlugin({ { "drive", robotik::mission<DuckMission>() },
                          { "tour", robotik::mission<DuckMission>() },
                          { "walk_forward", robotik::mission<DuckMission>() } })
    {
    }

    void setup(robotik::Host& p_host, std::string_view p_scenario) override
    {
        MissionPlugin::setup(p_host, p_scenario);
        m_lag = 0.0;
        p_host.addPanel("microduck", "Micro Duck",
                        [this](robotik::Canvas const& p_canvas) { draw(p_canvas); });
    }

    void update(robotik::Host& p_host, robotik::Seconds p_dt) override
    {
        DuckMission* duck = mission<DuckMission>();
        robotik::Simulation* simulation = p_host.simulation();
        if (duck == nullptr || simulation == nullptr)
        {
            return;
        }
        robotik::Seconds const step = duck->physicsStep();
        m_lag = std::min(m_lag + p_dt.value(), MAX_LAG_S);
        // The physics goes on after the end of the run, so the simulator
        // shows a fall. The verdict is already frozen; the headless host
        // stops at the end.
        while (m_lag + 1e-9 >= step.value())
        {
            simulation->step(step);
            m_lag -= step.value();
        }
    }

private:

    /// The walk command, and how far the duck went.
    void draw(robotik::Canvas const& p_canvas)
    {
        DuckMission* duck = mission<DuckMission>();
        if (duck == nullptr)
        {
            return;
        }
        // Sliders speak cm/s and deg/s; the command is SI.
        std::array<double, 3>& command = duck->command();
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
        p_canvas.text(format("Distance %.2f m", duck->distance()));
        if (duck->height() < FALLEN_HEIGHT_M)
        {
            p_canvas.textColored(0.90F, 0.25F, 0.25F, "The duck fell: Replay");
        }
        else
        {
            p_canvas.text(format("Trunk height %.3f m", duck->height()));
        }
    }

    /// Host time not yet simulated (s).
    double m_lag = 0.0;
};

ROBOTIK_EXPORT_PLUGIN(MicroDuckPlugin)
