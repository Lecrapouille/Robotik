// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "FlyMission.hpp"

#include "Robotik/Package/Plugin/PluginAPI.hpp"

#include <chrono>
#include <exception>
#include <future>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>

/// Loads the fly mission and its view.
class FlyPlugin final: public robotik::Plugin
{
public:

    static robotik::PluginInfo describe()
    {
        return { .id = "robotik.fly" };
    }

    RobotikPluginStatus setup(robotik::Host& p_host,
                              std::string_view p_scenario) override
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

    RobotikPluginStatus update(robotik::Host& /*p_host*/,
                               double /*p_dt*/) override
    {
        poll();
        return ROBOTIK_PLUGIN_OK;
    }

    RobotikPluginStatus shutdown(robotik::Host& /*p_host*/) override
    {
        // A load in progress finishes first; its brain is dropped.
        if (m_loading.valid())
        {
            m_loading.wait();
            m_loading = {};
        }
        m_mission.reset();
        return ROBOTIK_PLUGIN_OK;
    }

    void draw(robotik::Canvas const& p_canvas)
    {
        FlyEnvironment const* environment =
            m_mission ? m_mission->environment() : nullptr;
        FlyBrain const* brain = m_mission ? m_mission->brain() : nullptr;
        if (environment == nullptr || brain == nullptr)
        {
            p_canvas.text("Fly brain");
            return;
        }
        FlySnapshot const& snapshot = environment->snapshot();
        std::ostringstream state;
        state << std::fixed << std::setprecision(2)
              << (snapshot.reached ? "Food reached" : "Flying") << "  ·  step "
              << snapshot.steps << "  ·  at " << snapshot.plant.x << ", "
              << snapshot.plant.y << ", " << snapshot.plant.z << " m";
        if (snapshot.reached)
        {
            p_canvas.textColored(0.35F, 0.85F, 0.40F, state.str());
        }
        else
        {
            p_canvas.text(state.str());
        }
        FlyAction const& action = m_mission->action();
        std::ostringstream decided;
        decided << std::fixed << std::setprecision(2) << "Decision  forward "
                << action.forward << "   turn " << action.turn << "   lift "
                << action.lift;
        p_canvas.text(decided.str());
        std::ostringstream eyes;
        eyes << std::fixed << std::setprecision(2) << "Eyes  left "
             << snapshot.vision.left << "   center " << snapshot.vision.center
             << "   right " << snapshot.vision.right;
        p_canvas.text(eyes.str());
        p_canvas.separator();
        brainChoice(p_canvas, *brain);
    }

private:

    /// Installs a brain loaded in the background, once it is ready.
    void poll()
    {
        if (!m_loading.valid() || m_loading.wait_for(std::chrono::seconds(0)) !=
                                      std::future_status::ready)
        {
            return;
        }
        try
        {
            m_mission->installBrain(m_loading.get());
            m_error.clear();
        }
        catch (std::exception const& error)
        {
            m_error = std::string(error.what()) +
                      ". Run make -C plugins/FlyBrain "
                      "download-external-libs, then make";
        }
        m_loading = {};
    }

    void startLoading(bool p_connectome)
    {
        m_error.clear();
        m_target = p_connectome;
        m_started = std::chrono::steady_clock::now();
        // Reading the FlyWire tables takes seconds: not on the UI thread.
        m_loading = std::async(std::launch::async,
                               &FlyMission::buildBrain,
                               p_connectome,
                               m_mission->cruise(),
                               m_mission->tables());
    }

    void brainChoice(robotik::Canvas const& p_canvas, FlyBrain const& p_brain)
    {
        poll();
        p_canvas.text("Brain");
        bool const loading = m_loading.valid();
        bool const connectome = loading ? m_target : p_brain.connectome();
        if (p_canvas.radio("Small circuit (13 neurons, starts at once)",
                           !connectome) &&
            !loading && p_brain.connectome())
        {
            startLoading(false);
        }
        if (p_canvas.radio(
                "FlyWire connectome (whole fly brain, loads in a few seconds)",
                connectome) &&
            !loading && !p_brain.connectome())
        {
            startLoading(true);
        }
        if (loading)
        {
            auto const elapsed =
                std::chrono::duration_cast<std::chrono::seconds>(
                    std::chrono::steady_clock::now() - m_started);
            std::ostringstream text;
            text << "Loading the "
                 << (m_target ? "FlyWire connectome" : "small circuit")
                 << "... " << elapsed.count()
                 << " s  (the fly keeps its current brain meanwhile)";
            p_canvas.textColored(1.00F, 0.70F, 0.20F, text.str());
            return;
        }
        std::ostringstream size;
        size << "Active: "
             << (p_brain.kind() == FlyBrain::Kind::Reflex ? "reflex rule"
                                                          : "Shiu LIF network")
             << ", " << p_brain.neurons() << " neurons, " << p_brain.synapses()
             << " synapses";
        p_canvas.text(size.str());
        if (!m_error.empty())
        {
            p_canvas.textColored(0.90F, 0.25F, 0.25F, m_error);
        }
    }

    static void onDraw(void* p_user, RobotikCanvas const* p_canvas)
    {
        static_cast<FlyPlugin*>(p_user)->draw(robotik::Canvas(p_canvas));
    }

    std::unique_ptr<FlyMission> m_mission;
    std::string m_error;
    std::future<std::unique_ptr<FlyBrain>> m_loading;
    bool m_target = false;
    std::chrono::steady_clock::time_point m_started;
};

ROBOTIK_EXPORT_PLUGIN(FlyPlugin)
