// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

// One task, several controllers: a behavior tree, the PDDL planner (both
// with and without faults) and a reinforcement learning policy. The scenario
// id picks the mission.

#include "PickPlaceMission.hpp"
#include "RlMission.hpp"

#include "Robotik/Plugin/PluginAPI.hpp"
#include "Robotik/Runtime/Simulation.hpp"

#include <memory>
#include <sstream>
#include <string>
#include <string_view>

namespace
{

class PickPlacePlugin final: public robotik::Plugin
{
public:

    static robotik::PluginInfo describe()
    {
        return { .id = "robotik.pick_and_place" };
    }

    RobotikPluginStatus setup(robotik::Host& p_host, std::string_view p_scenario) override
    {
        m_host = p_host;
        m_tree.reset();
        m_rl.reset();
        if (p_scenario == "pick_and_place" || p_scenario == "pick_and_place_faults")
        {
            m_tree = std::make_unique<PickPlaceMission>(true);
            p_host.bindMission(*m_tree);
            return ROBOTIK_PLUGIN_OK;
        }
        if (p_scenario == "pick_and_place_pddl" || p_scenario == "pick_and_place_pddl_faults")
        {
            m_tree = std::make_unique<PickPlaceMission>(true, true);
            p_host.bindMission(*m_tree);
            return ROBOTIK_PLUGIN_OK;
        }
        if (p_scenario == "pick_and_place_rl")
        {
            m_rl = std::make_unique<RlMission>();
            p_host.bindMission(*m_rl);
            p_host.addPanel("rl", "RL", &onDraw, this);
            return ROBOTIK_PLUGIN_OK;
        }
        p_host.error("unknown pick-and-place scenario");
        return ROBOTIK_PLUGIN_ERR_SETUP;
    }

    //! The RL scenario owns the clock (plugin.yaml): one policy action per
    //! update. The behavior tree scenarios are stepped by the simulation.
    RobotikPluginStatus update(robotik::Host& p_host, double p_dt) override
    {
        if (m_rl == nullptr)
        {
            return ROBOTIK_PLUGIN_OK;
        }
        if (robotik::Simulation* simulation = p_host.simulation())
        {
            m_rl->act(*simulation, p_dt);
        }
        return ROBOTIK_PLUGIN_OK;
    }

    RobotikPluginStatus shutdown(robotik::Host& /*p_host*/) override
    {
        m_tree.reset();
        m_rl.reset();
        return ROBOTIK_PLUGIN_OK;
    }

    void draw(robotik::Canvas const& p_canvas)
    {
        if (m_rl == nullptr)
        {
            p_canvas.text("Pick and place RL");
            return;
        }
        p_canvas.text("Converged completes the pick. Random converts toward that recipe.");
        if (p_canvas.checkbox("Converged", m_rl->converged) && m_rl->converged)
        {
            m_rl->mix = 1.0f;
        }
        bool random = !m_rl->converged;
        if (p_canvas.checkbox("Random", random))
        {
            m_rl->converged = !random;
            if (random)
            {
                m_rl->mix = 0.0f;
            }
        }
        std::ostringstream mix;
        mix << (m_rl->converged ? "converged" : "random to converged") << "  mix " << m_rl->mix;
        p_canvas.text(mix.str());
        (void)p_canvas.checkbox("Repeat episodes", m_rl->repeat);
        (void)p_canvas.sliderInt("Max steps", m_rl->max_steps, 40, 200);
        if (p_canvas.button("New episode"))
        {
            if (robotik::Simulation* simulation = m_host.simulation())
            {
                simulation->reset(robotik::Seed{ simulation->seed().value + 1u });
            }
        }
        p_canvas.separator();
        p_canvas.text(std::string("Phase ") + policyPhase(m_rl->observation));
        std::ostringstream score;
        score << "Return " << m_rl->episode_return << "   steps " << m_rl->steps << " / "
              << m_rl->max_steps;
        p_canvas.text(score.str());
        std::ostringstream tally;
        tally << m_rl->delivered << " delivered / " << m_rl->failed << " failed";
        p_canvas.text(tally.str());
        if (!m_rl->ready)
        {
            p_canvas.textColored(0.90f, 0.25f, 0.25f, "Needs an arm joint group and a vacuum gripper");
        }
        else if (m_rl->success)
        {
            p_canvas.textColored(0.35f, 0.85f, 0.40f, "Delivered — cube is in the box");
        }
        else if (m_rl->done)
        {
            p_canvas.textColored(0.90f, 0.25f, 0.25f, "Truncated — out of steps");
        }
        else
        {
            p_canvas.text("Running");
        }
    }

private:

    static void onDraw(void* p_user, RobotikCanvas const* p_canvas)
    {
        static_cast<PickPlacePlugin*>(p_user)->draw(robotik::Canvas(p_canvas));
    }

    robotik::Host m_host;
    std::unique_ptr<PickPlaceMission> m_tree;
    std::unique_ptr<RlMission> m_rl;
};

} // namespace

ROBOTIK_EXPORT_PLUGIN(PickPlacePlugin)
