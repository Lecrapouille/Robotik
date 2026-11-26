// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "RobotIK/Package/Plugin/PluginAPI.hpp"
#include "RobotIK/Task/RoboticCell.hpp"
#include "RobotIK/Task/ScenarioLogic.hpp"

// ****************************************************************************
/// The hall of a reception robot. The people are moved by the story of the
/// scenario, not by this code: the logic only publishes what a test of the
/// robot will judge. The robot has no skill yet (see ROADMAP-HRI.md).
// ****************************************************************************
class ReceptionLogic final: public robotik::ScenarioLogic
{
public:

    /// People the robot registered, read from the story.
    void measure(robotik::RoboticCell const& p_cell, robotik::Metrics& p_metrics) const override
    {
        p_metrics.set("reception.registered",
                      p_cell.requireQuery("story_var", { "registered" }).value_or(0.0));
    }
};

// ****************************************************************************
/// The plugin: one scenario logic per scenario of plugin.yaml.
// ****************************************************************************
class ReceptionPlugin final: public robotik::ScenarioPlugin
{
public:

    /// The id must match plugin.id in plugin.yaml.
    static robotik::PluginInfo describe()
    {
        return { .id = "robotik.reception" };
    }

    ReceptionPlugin()
        : ScenarioPlugin({ { "welcome", robotik::logic<ReceptionLogic>() } })
    {
    }
};

ROBOTIK_EXPORT_PLUGIN(ReceptionPlugin)
