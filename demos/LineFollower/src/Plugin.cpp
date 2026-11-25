// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "LineFollowerLogic.hpp"

#include "RobotIK/Package/Plugin/PluginAPI.hpp"

#include <sstream>

/// Loads the line-follower scenario logic and shows its navigation.
class LineFollowerPlugin final: public robotik::ScenarioPlugin
{
public:

    static robotik::PluginInfo describe()
    {
        return { .id = "robotik.line_follower" };
    }

    LineFollowerPlugin()
        : ScenarioPlugin(
              { { "line_follower", robotik::logic<LineFollowerLogic>(1.0) } })
    {
    }

    void setup(robotik::Host& p_host, std::string_view p_scenario) override
    {
        ScenarioPlugin::setup(p_host, p_scenario);
        p_host.addPanel("line", "Line follower",
                        [this](robotik::Canvas const& p_canvas) { draw(p_canvas); });
    }

private:

    /// Fixes, laps and the estimate against the truth.
    void draw(robotik::Canvas const& p_canvas) const
    {
        LineFollowerLogic const* line = logic<LineFollowerLogic>();
        if (line == nullptr)
        {
            p_canvas.text("Line follower");
            return;
        }
        std::ostringstream fixes;
        fixes << "Fixes " << line->navigation().fixes << "  rejected "
              << line->navigation().rejected << "  error mean "
              << (1000.0 * line->fixErrorMean()) << " mm";
        p_canvas.text(fixes.str());
        double const perimeter = line->track().perimeter();
        std::ostringstream laps;
        laps << "Laps "
             << (perimeter > 0.0 ? line->followProgress() / perimeter
                                 : 0.0)
             << "  cross-track max " << (1000.0 * line->crossTrackMax())
             << " mm";
        p_canvas.text(laps.str());
        if (line->drive() != nullptr)
        {
            Pose2 const& truth = line->drive()->truth();
            std::ostringstream pose;
            pose << "Truth " << truth.x << " " << truth.y << "   estimate "
                 << line->navigation().estimate.x << " "
                 << line->navigation().estimate.y;
            p_canvas.text(pose.str());
        }
    }
};

ROBOTIK_EXPORT_PLUGIN(LineFollowerPlugin)
