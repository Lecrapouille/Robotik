// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "Robotik/Runtime/Faults.hpp"

#include <algorithm>
#include <cmath>

namespace robotik
{

FaultInjector::FaultInjector(std::vector<Fault> p_scheduled,
                             std::vector<RandomFault> p_random)
    : m_scheduled(std::move(p_scheduled)),
      m_random_faults(std::move(p_random))
{
    std::stable_sort(m_scheduled.begin(),
                     m_scheduled.end(),
                     [](Fault const& p_a, Fault const& p_b)
                     { return p_a.at < p_b.at; });
}

void FaultInjector::reset(Seed p_seed)
{
    m_next = 0;
    m_random = Random(p_seed);
}

void FaultInjector::update(ResourceManager& p_resources,
                           Seconds p_now,
                           Seconds p_dt)
{
    for (; m_next < m_scheduled.size() && m_scheduled[m_next].at <= p_now;
         ++m_next)
    {
        Fault const& fault = m_scheduled[m_next];
        if (fault.disable)
        {
            p_resources.fail(fault.resource);
        }
        else
        {
            p_resources.restore(fault.resource);
        }
    }

    for (RandomFault const& fault : m_random_faults)
    {
        // Poisson process: probability of at least one event during dt.
        double const probability = 1.0 - std::exp(-fault.rate * p_dt.value());
        if (m_random.chance(probability))
        {
            p_resources.fail(fault.resource);
        }
    }
}

} // namespace robotik
