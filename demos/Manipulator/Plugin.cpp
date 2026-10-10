// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "ManipulatorMission.hpp"

#include "Robotik/Plugin/PluginAPI.hpp"

#include <memory>
#include <string>

namespace
{

class ManipulatorPlugin final: public robotik::Plugin
{
public:

    static robotik::PluginInfo describe()
    {
        return { "robotik.manipulator", "Manipulator", "1.0.0", "Kinematics" };
    }

    RobotikPluginStatus setup(robotik::Host& p_host, std::string_view p_scenario) override
    {
        m_host = p_host;
        m_scenario = std::string(p_scenario);
        m_motion.reset();
        m_assertion.clear();
        m_assertion_ok = false;

        if (p_scenario == "forward_kinematics")
        {
            m_motion = std::make_unique<ManipulatorMission>(ManipulatorMission::Mode::Forward);
        }
        else if (p_scenario == "inverse_kinematics")
        {
            m_motion = std::make_unique<ManipulatorMission>(ManipulatorMission::Mode::Inverse);
        }
        else if (p_scenario == "joint_limits")
        {
            m_motion = std::make_unique<ManipulatorMission>(ManipulatorMission::Mode::Limits);
        }
        else
        {
            p_host.error("unknown manipulator scenario");
            return ROBOTIK_PLUGIN_ERR_SETUP;
        }

        p_host.bindMission(*m_motion);
        p_host.subscribe(ROBOTIK_EVENT_KEY_PRESSED);
        p_host.subscribe(ROBOTIK_EVENT_ASSERTION_CHANGED);
        p_host.subscribe(ROBOTIK_EVENT_SKILL_COMPLETED);
        p_host.addMenuItem("Manipulator", "Home", &onHome, this);
        p_host.addPanel("manipulator", "Manipulator", &onDraw, this);
        return ROBOTIK_PLUGIN_OK;
    }

    RobotikPluginStatus onEvent(robotik::Host& /*p_host*/,
                                RobotikEvent const& p_event) override
    {
        if (p_event.kind == ROBOTIK_EVENT_KEY_PRESSED && p_event.code == 'H')
        {
            home();
        }
        else if (p_event.kind == ROBOTIK_EVENT_ASSERTION_CHANGED)
        {
            m_assertion = p_event.name;
            m_assertion_ok = p_event.flag != 0;
        }
        return ROBOTIK_PLUGIN_OK;
    }

    RobotikPluginStatus shutdown(robotik::Host& /*p_host*/) override
    {
        m_motion.reset();
        return ROBOTIK_PLUGIN_OK;
    }

    void home()
    {
        if (m_motion)
        {
            m_motion->requestHome();
        }
    }

    void draw(robotik::Canvas const& p_canvas) const
    {
        if (m_motion)
        {
            p_canvas.text(std::string(m_motion->modeName()));
            p_canvas.text("samples " + std::to_string(static_cast<int>(m_motion->samples())) +
                          "  ik error " + std::to_string(m_motion->error()));
        }
        if (!m_assertion.empty())
        {
            p_canvas.textColored(m_assertion_ok ? 0.35f : 0.90f,
                                 m_assertion_ok ? 0.85f : 0.25f,
                                 m_assertion_ok ? 0.40f : 0.25f,
                                 (m_assertion_ok ? "[ok] " : "[ko] ") + m_assertion);
        }
    }

private:

    static void onHome(void* p_user)
    {
        static_cast<ManipulatorPlugin*>(p_user)->home();
    }

    static void onDraw(void* p_user, RobotikCanvas const* p_canvas)
    {
        static_cast<ManipulatorPlugin*>(p_user)->draw(robotik::Canvas(p_canvas));
    }

    robotik::Host m_host;
    std::string m_scenario;
    std::string m_assertion;
    bool m_assertion_ok = false;
    std::unique_ptr<ManipulatorMission> m_motion;
};

} // namespace

ROBOTIK_EXPORT_PLUGIN(ManipulatorPlugin)
