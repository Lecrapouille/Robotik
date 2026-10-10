// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "LineFollowerMission.hpp"

#include "Robotik/Plugin/PluginAPI.hpp"
#include "Robotik/Runtime/Simulation.hpp"

#include <memory>
#include <sstream>
#include <string>
#include <string_view>

namespace
{

class LineFollowerPlugin final: public robotik::Plugin
{
public:

    static robotik::PluginInfo describe()
    {
        return { "robotik.line_follower", "Line follower", "1.0.0", "Mobile" };
    }

    RobotikPluginStatus setup(robotik::Host& p_host, std::string_view p_scenario) override
    {
        m_host = p_host;
        m_mission.reset();
        if (p_scenario != "line_follower")
        {
            p_host.error("unknown line follower scenario");
            return ROBOTIK_PLUGIN_ERR_SETUP;
        }
        m_mission = std::make_unique<LineFollowerMission>(1.0);
        p_host.bindMission(*m_mission);
        p_host.addPanel("line", "Line follower", &onDraw, this);
        return ROBOTIK_PLUGIN_OK;
    }

    RobotikPluginStatus shutdown(robotik::Host& /*p_host*/) override
    {
        m_mission.reset();
        return ROBOTIK_PLUGIN_OK;
    }

    void draw(robotik::Canvas const& p_canvas) const
    {
        if (m_mission == nullptr)
        {
            p_canvas.text("Line follower");
            return;
        }
        std::ostringstream fixes;
        fixes << "Fixes " << m_mission->navigation().fixes
              << "  rejected " << m_mission->navigation().rejected
              << "  error mean " << (1000.0 * m_mission->fixErrorMean()) << " mm";
        p_canvas.text(fixes.str());
        double const perimeter = m_mission->track().perimeter();
        std::ostringstream laps;
        laps << "Laps " << (perimeter > 0.0 ? m_mission->followProgress() / perimeter : 0.0)
             << "  cross-track max " << (1000.0 * m_mission->crossTrackMax()) << " mm";
        p_canvas.text(laps.str());
        if (m_mission->drive() != nullptr)
        {
            Pose2 const& truth = m_mission->drive()->truth();
            std::ostringstream pose;
            pose << "Truth " << truth.x << " " << truth.y
                 << "   estimate " << m_mission->navigation().estimate.x
                 << " " << m_mission->navigation().estimate.y;
            p_canvas.text(pose.str());
        }
    }

private:

    static void onDraw(void* p_user, RobotikCanvas const* p_canvas)
    {
        static_cast<LineFollowerPlugin*>(p_user)->draw(robotik::Canvas(p_canvas));
    }

    robotik::Host m_host;
    std::unique_ptr<LineFollowerMission> m_mission;
};

} // namespace

ROBOTIK_EXPORT_PLUGIN(LineFollowerPlugin)
