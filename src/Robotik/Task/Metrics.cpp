// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "Robotik/Task/Metrics.hpp"

namespace robotik
{

void Metrics::set(std::string_view p_name, double p_value)
{
    for (std::size_t i = 0; i < m_names.size(); ++i)
    {
        if (m_names[i] == p_name)
        {
            m_values[i] = p_value;
            return;
        }
    }
    m_names.emplace_back(p_name);
    m_values.push_back(p_value);
}

std::optional<double> Metrics::get(std::string_view p_name) const
{
    for (std::size_t i = 0; i < m_names.size(); ++i)
    {
        if (m_names[i] == p_name)
        {
            return m_values[i];
        }
    }
    return std::nullopt;
}

} // namespace robotik
