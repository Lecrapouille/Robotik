// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "RlMission.hpp"

#include "PickPlaceControl.hpp"

#include "Robotik/Plugin/PluginAPI.hpp"
#include "Robotik/Runtime/Simulation.hpp"

#include <memory>
#include <sstream>
#include <string>
#include <string_view>

namespace
{

class PickPlaceRlPlugin final: public robotik::Plugin
{
public:

    static robotik::PluginInfo describe()
    {
        return { "robotik.pick_and_place_rl", "Pick and place RL", "1.0.0", "Learning" };
    }

    RobotikPluginStatus setup(robotik::Host& p_host, std::string_view p_scenario) override
    {
        m_host = p_host;
        m_mission.reset();
        if (p_scenario != "pick_and_place_rl")
        {
            p_host.error("unknown reinforcement learning scenario");
            return ROBOTIK_PLUGIN_ERR_SETUP;
        }
        m_mission = std::make_unique<RlMission>();
        p_host.bindMission(*m_mission);
        p_host.addPanel("rl", "RL", &onDraw, this);
        return ROBOTIK_PLUGIN_OK;
    }

    RobotikPluginStatus update(robotik::Host& p_host, double p_dt) override
    {
        if (m_mission == nullptr)
        {
            return ROBOTIK_PLUGIN_OK;
        }
        if (robotik::Simulation* simulation = p_host.simulation())
        {
            m_mission->act(*simulation, p_dt);
        }
        return ROBOTIK_PLUGIN_OK;
    }

    RobotikPluginStatus shutdown(robotik::Host& /*p_host*/) override
    {
        m_mission.reset();
        return ROBOTIK_PLUGIN_OK;
    }

    void draw(robotik::Canvas const& p_canvas)
    {
        if (m_mission == nullptr)
        {
            p_canvas.text("Pick and place RL");
            return;
        }
        p_canvas.text("Converged completes the pick. Random converts toward that recipe.");
        if (p_canvas.checkbox("Converged", m_mission->converged) && m_mission->converged)
        {
            m_mission->mix = 1.0f;
        }
        bool random = !m_mission->converged;
        if (p_canvas.checkbox("Random", random))
        {
            m_mission->converged = !random;
            if (random)
            {
                m_mission->mix = 0.0f;
            }
        }
        std::ostringstream mix;
        mix << (m_mission->converged ? "converged" : "random to converged")
            << "  mix " << m_mission->mix;
        p_canvas.text(mix.str());
        p_canvas.checkbox("Repeat episodes", m_mission->repeat);
        p_canvas.sliderInt("Max steps", m_mission->max_steps, 40, 200);
        if (p_canvas.button("New episode"))
        {
            if (robotik::Simulation* simulation = m_host.simulation())
            {
                simulation->reset(robotik::Seed{ simulation->seed().value + 1u });
            }
        }
        p_canvas.separator();
        p_canvas.text(std::string("Phase ") + policyPhase(m_mission->observation));
        std::ostringstream score;
        score << "Return " << m_mission->episode_return
              << "   steps " << m_mission->steps << " / " << m_mission->max_steps;
        p_canvas.text(score.str());
        std::ostringstream tally;
        tally << m_mission->delivered << " delivered / " << m_mission->failed << " failed";
        p_canvas.text(tally.str());
        if (!m_mission->ready)
        {
            p_canvas.textColored(0.90f, 0.25f, 0.25f, "Needs an arm joint group and a vacuum gripper");
        }
        else if (m_mission->success)
        {
            p_canvas.textColored(0.35f, 0.85f, 0.40f, "Delivered — cube is in the box");
        }
        else if (m_mission->done)
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
        static_cast<PickPlaceRlPlugin*>(p_user)->draw(robotik::Canvas(p_canvas));
    }

    robotik::Host m_host;
    std::unique_ptr<RlMission> m_mission;
};

} // namespace

ROBOTIK_EXPORT_PLUGIN(PickPlaceRlPlugin)
