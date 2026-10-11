// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "Robotik/Plugin/PluginAPI.hpp"

namespace
{

class FlyPlugin final: public robotik::Plugin
{
public:

    static robotik::PluginInfo describe()
    {
        return { .id = "robotik.fly" };
    }

    RobotikPluginStatus setup(robotik::Host& p_host, std::string_view /*p_scenario*/) override
    {
        p_host.error("the fly view is hosted by the simulator");
        return ROBOTIK_PLUGIN_ERR_SETUP;
    }
};

} // namespace

ROBOTIK_EXPORT_PLUGIN(FlyPlugin)
