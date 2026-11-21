// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "Robotik/Package/Plugin/PluginAPI.hpp"

#include "Robotik/Package/Simulation.hpp"
#include "Robotik/Task/ScenarioLogic.hpp"

namespace robotik
{

Simulation* Host::simulation() const
{
    if (m_api == nullptr || m_api->simulation == nullptr ||
        m_api->host == nullptr)
    {
        return nullptr;
    }
    return static_cast<Simulation*>(m_api->simulation(m_api->host));
}

void Host::bindLogic(ScenarioLogic& p_logic) const
{
    if (m_api != nullptr && m_api->bind_logic != nullptr &&
        m_api->host != nullptr)
    {
        m_api->bind_logic(m_api->host, &p_logic);
    }
}

} // namespace robotik
