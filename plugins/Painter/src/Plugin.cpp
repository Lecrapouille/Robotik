// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "PainterMission.hpp"

#include "Robotik/Package/Plugin/PluginAPI.hpp"

/// The painter mission; the planned scenarios let the planner choose the
/// order of the layers.
class PainterPlugin final: public robotik::MissionPlugin
{
public:

    static robotik::PluginInfo describe()
    {
        return { .id = "robotik.painter" };
    }

    PainterPlugin()
        : MissionPlugin({ { "painter", robotik::mission<PainterMission>(false) },
                          { "painter_pddl", robotik::mission<PainterMission>(true) },
                          { "painter_pabt", robotik::mission<PainterMission>(true) } })
    {
    }
};

ROBOTIK_EXPORT_PLUGIN(PainterPlugin)
