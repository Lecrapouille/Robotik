// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "PainterLogic.hpp"

#include "RobotIK/Package/Plugin/PluginAPI.hpp"

/// The painter scenario logic; the planned scenarios let the planner choose the
/// order of the layers.
class PainterPlugin final: public robotik::ScenarioPlugin
{
public:

    static robotik::PluginInfo describe()
    {
        return { .id = "robotik.painter" };
    }

    PainterPlugin()
        : ScenarioPlugin(
              { { "painter", robotik::logic<PainterLogic>(false) },
                { "painter_pddl", robotik::logic<PainterLogic>(true) },
                { "painter_pabt", robotik::logic<PainterLogic>(true) } })
    {
    }
};

ROBOTIK_EXPORT_PLUGIN(PainterPlugin)
