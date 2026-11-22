// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "WalkLogic.hpp"

#include "RobotIK/Package/Plugin/PluginAPI.hpp"
#include "RobotIK/Package/Simulation.hpp"

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <numbers>
#include <string>

/// Largest host time simulated in one update (s): a slow frame does not pile
/// up a lag the simulation never catches up.
#define MAX_LAG_S 0.05

/// Prints numbers with a printf format, checked by the compiler.
[[gnu::format(printf, 1, 2)]] static std::string format(char const* p_format,
                                                        ...)
{
    char line[96];
    va_list args;
    va_start(args, p_format);
    std::vsnprintf(line, sizeof(line), p_format, args);
    va_end(args);
    return line;
}

// ****************************************************************************
/// Humanoid walking by a learned policy. The plugin owns the clock
/// (host.clock: plugin): it steps the simulation at the physics rate of the
/// training, whatever the host rate, so the policy runs at its own rate.
/// Its panel changes the walk command while the robot walks.
// ****************************************************************************
class HumanoidWalkPlugin final: public robotik::ScenarioPlugin
{
public:

    static robotik::PluginInfo describe()
    {
        return { .id = "robotik.humanoid_walk" };
    }

    HumanoidWalkPlugin()
        : ScenarioPlugin({ { "walk", robotik::logic<WalkLogic>() },
                           { "walk_circle", robotik::logic<WalkLogic>() } })
    {
    }

    void setup(robotik::Host& p_host, std::string_view p_scenario) override
    {
        ScenarioPlugin::setup(p_host, p_scenario);
        m_lag = 0.0;
        p_host.addPanel("humanoid", "Humanoid",
                        [this](robotik::Canvas const& p_canvas) { draw(p_canvas); });
    }

    void update(robotik::Host& p_host, robotik::Seconds p_dt) override
    {
        robotik::Simulation* simulation = p_host.simulation();
        if (logic() == nullptr || simulation == nullptr)
        {
            return;
        }
        robotik::Seconds const step = WalkLogic::physicsStep();
        m_lag = std::min(m_lag + p_dt.value(), MAX_LAG_S);
        // The physics goes on after the end of the run, so the simulator
        // shows what follows a failure (a robot without IMU falls). The
        // verdict is already frozen; the headless host stops at the end.
        // The epsilon absorbs rounding: 0.01 s is ten steps, not nine.
        while (m_lag + 1e-9 >= step.value())
        {
            simulation->step(step);
            m_lag -= step.value();
        }
    }

private:

    /// The walk command, and how the robot follows it.
    void draw(robotik::Canvas const& p_canvas)
    {
        WalkLogic* walk = logic<WalkLogic>();
        if (walk == nullptr)
        {
            return;
        }
        // Sliders speak cm/s and deg/s; the command is SI.
        std::array<double, 3>& command = walk->command();
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
            walk->radius(),
            walk->commandedRadius()));
        p_canvas.text(format("Turns %.2f   distance %.1f m",
                             walk->turns(),
                             walk->distance()));
        if (walk->height() < 0.6)
        {
            p_canvas.textColored(0.90F, 0.25F, 0.25F, "The robot fell: Replay");
        }
        else
        {
            p_canvas.text(format("Pelvis height %.2f m", walk->height()));
        }
    }

    /// Host time not yet simulated (s).
    double m_lag = 0.0;
};

ROBOTIK_EXPORT_PLUGIN(HumanoidWalkPlugin)
