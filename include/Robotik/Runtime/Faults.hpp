// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

//! @file Faults.hpp
//! @brief Scripted and random failures applied to robot resources.
//!
//! The injector only flips resource availability; skills never know whether
//! a failure was injected, simulated or real.
#pragma once

#include "Robotik/Math/Random.hpp"
#include "Robotik/Runtime/Resources.hpp"

#include "Compages/Core/Units.hpp"

#include <string>
#include <vector>

namespace robotik
{

// ****************************************************************************
//! @brief A failure (or a repair) at a given time.
// ****************************************************************************
struct Fault
{
    Seconds at{};
    std::string resource;
    //!< True disables the resource, false restores it.
    bool disable = true;
};

// ****************************************************************************
//! @brief A resource failing at random, @c rate times per second on average.
// ****************************************************************************
struct RandomFault
{
    std::string resource;
    double rate = 0.0;
};

// ****************************************************************************
//! @brief Applies a fault plan to a @ref ResourceManager over time.
// ****************************************************************************
class FaultInjector
{
public:

    FaultInjector() = default;
    FaultInjector(std::vector<Fault> p_scheduled,
                  std::vector<RandomFault> p_random);

    // -------------------------------------------------------------------------
    //! @brief Rewinds the plan; @p_seed drives the random faults.
    // -------------------------------------------------------------------------
    void reset(Seed p_seed);

    // -------------------------------------------------------------------------
    //! @brief Applies the faults due in (@p_now - @p_dt, @p_now].
    // -------------------------------------------------------------------------
    void update(ResourceManager& p_resources, Seconds p_now, Seconds p_dt);

    [[nodiscard]] std::vector<Fault> const& scheduled() const
    {
        return m_scheduled;
    }

    [[nodiscard]] std::vector<RandomFault> const& random() const
    {
        return m_random_faults;
    }

private:

    std::vector<Fault> m_scheduled;
    std::vector<RandomFault> m_random_faults;
    std::size_t m_next = 0;
    Random m_random;
};

} // namespace robotik
