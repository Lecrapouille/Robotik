// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "FlyMission.hpp"

#include "Robotik/Plugin/PluginAPI.hpp"

#include <exception>
#include <memory>
#include <sstream>
#include <string>

namespace
{

class FlyPlugin final: public robotik::Plugin
{
public:

    static robotik::PluginInfo describe()
    {
        return { .id = "robotik.fly" };
    }

    RobotikPluginStatus setup(robotik::Host& p_host, std::string_view p_scenario) override
    {
        if (p_scenario != "fly_obstacle_avoidance")
        {
            p_host.error("unknown fly scenario");
            return ROBOTIK_PLUGIN_ERR_SETUP;
        }
        m_mission = std::make_unique<FlyMission>();
        m_error.clear();
        p_host.bindMission(*m_mission);
        p_host.addPanel("fly", "Fly brain", &onDraw, this);
        return ROBOTIK_PLUGIN_OK;
    }

    RobotikPluginStatus shutdown(robotik::Host& /*p_host*/) override
    {
        m_mission.reset();
        return ROBOTIK_PLUGIN_OK;
    }

    void draw(robotik::Canvas const& p_canvas)
    {
        FlyEnvironment const* environment = m_mission ? m_mission->environment() : nullptr;
        FlyBrain const* brain = m_mission ? m_mission->brain() : nullptr;
        if (environment == nullptr || brain == nullptr)
        {
            p_canvas.text("Fly brain");
            return;
        }
        FlySnapshot const& snapshot = environment->snapshot();
        std::ostringstream state;
        state << (snapshot.reached ? "FOOD" : "FLY") << "  step " << snapshot.steps
              << "  x " << snapshot.plant.x << "  y " << snapshot.plant.y
              << "  z " << snapshot.plant.z;
        if (snapshot.reached)
        {
            p_canvas.textColored(0.35f, 0.85f, 0.40f, state.str());
        }
        else
        {
            p_canvas.text(state.str());
        }
        FlyAction const& action = m_mission->action();
        std::ostringstream decided;
        decided << "forward " << action.forward << "  turn " << action.turn << "  lift "
                << action.lift;
        p_canvas.text(decided.str());
        std::ostringstream eyes;
        eyes << "eyes L " << snapshot.vision.left << "  C " << snapshot.vision.center
             << "  R " << snapshot.vision.right;
        p_canvas.text(eyes.str());
        p_canvas.separator();
        std::ostringstream size;
        size << (brain->kind() == FlyBrain::Kind::Reflex ? "reflex" : "Shiu LIF") << ": "
             << brain->neurons() << " neurons, " << brain->synapses() << " synapses";
        p_canvas.text(size.str());
        bool connectome = brain->connectome();
        if (p_canvas.checkbox("FlyWire connectome", connectome))
        {
            try
            {
                m_mission->useConnectome(connectome);
                m_error.clear();
            }
            catch (std::exception const& error)
            {
                m_error = std::string(error.what()) +
                          ". Run make download-external-libs && make compile-external-libs";
            }
        }
        if (!m_error.empty())
        {
            p_canvas.textColored(0.90f, 0.25f, 0.25f, m_error);
        }
    }

private:

    static void onDraw(void* p_user, RobotikCanvas const* p_canvas)
    {
        static_cast<FlyPlugin*>(p_user)->draw(robotik::Canvas(p_canvas));
    }

    std::unique_ptr<FlyMission> m_mission;
    std::string m_error;
};

} // namespace

ROBOTIK_EXPORT_PLUGIN(FlyPlugin)
