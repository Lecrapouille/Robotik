// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

// One task, several controllers: a behavior tree, the PDDL planner (both
// with and without faults) and a reinforcement learning policy. The scenario
// id picks the scenario logic.

#include "PickPlaceLogic.hpp"
#include "RlLogic.hpp"

#include "RobotIK/Package/Plugin/PluginAPI.hpp"
#include "RobotIK/Package/Simulation.hpp"

#include <sstream>
#include <string>

/// Loads the pick-and-place scenario logic chosen by the scenario.
class PickPlacePlugin final: public robotik::ScenarioPlugin
{
public:

    static robotik::PluginInfo describe()
    {
        return { .id = "robotik.pick_and_place" };
    }

    PickPlacePlugin()
        : ScenarioPlugin(
              { { "pick_and_place", robotik::logic<PickPlaceLogic>(true) },
                { "pick_and_place_faults",
                  robotik::logic<PickPlaceLogic>(true) },
                { "pick_and_place_pddl",
                  robotik::logic<PickPlaceLogic>(true, true) },
                { "pick_and_place_pddl_faults",
                  robotik::logic<PickPlaceLogic>(true, true) },
                { "pick_and_place_pabt",
                  robotik::logic<PickPlaceLogic>(true, true) },
                { "pick_and_place_pabt_faults",
                  robotik::logic<PickPlaceLogic>(true, true) },
                { "pick_and_place_rl", robotik::logic<RlLogic>() } })
    {
    }

    void setup(robotik::Host& p_host, std::string_view p_scenario) override
    {
        ScenarioPlugin::setup(p_host, p_scenario);
        m_host = p_host;
        if (logic<RlLogic>() != nullptr)
        {
            p_host.addPanel("rl", "RL",
                            [this](robotik::Canvas const& p_canvas) { draw(p_canvas); });
        }
    }

    /// The RL scenario owns the clock (plugin.yaml): one policy action per
    /// update. The behavior tree scenarios are stepped by the simulation.
    void update(robotik::Host& p_host, robotik::Seconds p_dt) override
    {
        RlLogic* rl = logic<RlLogic>();
        robotik::Simulation* simulation = p_host.simulation();
        if (rl != nullptr && simulation != nullptr)
        {
            rl->playPolicy(simulation->cell(), p_dt.value());
        }
    }

private:

    /// The policy, its switches and the score of the episode.
    void draw(robotik::Canvas const& p_canvas)
    {
        RlLogic* rl = logic<RlLogic>();
        if (rl == nullptr)
        {
            p_canvas.text("Pick and place RL");
            return;
        }
        p_canvas.text("Converged completes the pick. Random converts toward "
                      "that recipe.");
        if (p_canvas.checkbox("Converged", rl->converged) && rl->converged)
        {
            rl->mix = 1.0F;
        }
        bool random = !rl->converged;
        if (p_canvas.checkbox("Random", random))
        {
            rl->converged = !random;
            if (random)
            {
                rl->mix = 0.0F;
            }
        }
        std::ostringstream mix;
        mix << (rl->converged ? "converged" : "random to converged")
            << "  mix " << rl->mix;
        p_canvas.text(mix.str());
        (void)p_canvas.checkbox("Repeat episodes", rl->repeat);
        (void)p_canvas.sliderInt("Max steps", rl->max_steps, 40, 200);
        if (p_canvas.button("New episode"))
        {
            if (robotik::Simulation* simulation = m_host.simulation())
            {
                simulation->reset(
                    robotik::Seed{ simulation->seed().value + 1U });
            }
        }
        p_canvas.separator();
        p_canvas.text(std::string("Phase ") + policyPhase(rl->observation));
        std::ostringstream score;
        score << "Return " << rl->episode_return << "   steps " << rl->steps
              << " / " << rl->max_steps;
        p_canvas.text(score.str());
        std::ostringstream tally;
        tally << rl->delivered << " delivered / " << rl->failed
              << " failed";
        p_canvas.text(tally.str());
        if (!rl->ready)
        {
            p_canvas.textColored(
                0.90F,
                0.25F,
                0.25F,
                "Needs an arm joint group and a vacuum gripper");
        }
        else if (rl->success)
        {
            p_canvas.textColored(
                0.35F, 0.85F, 0.40F, "Delivered — cube is in the box");
        }
        else if (rl->done)
        {
            p_canvas.textColored(
                0.90F, 0.25F, 0.25F, "Truncated — out of steps");
        }
        else
        {
            p_canvas.text("Running");
        }
    }

    /// Kept to start a new episode from the panel.
    robotik::Host m_host;
};

ROBOTIK_EXPORT_PLUGIN(PickPlacePlugin)
