// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file Faults.hpp
/// Scripted and random failures applied to robot resources.
///
/// The injector only flips resource availability; skills never know whether
/// a failure was injected, simulated or real.
#pragma once

#include "Robotik/Math/Random.hpp"
#include "Robotik/Runtime/Resources.hpp"

#include "Robotik/Math/Si.hpp"

#include <string>
#include <vector>

namespace robotik
{

// ****************************************************************************
/// A failure (or a repair) at a given time.
///
/// @code
/// robotik::Fault const lose{ Seconds(3.0), "camera", true };
/// robotik::Fault const repair{ Seconds(5.0), "camera", false };
/// @endcode
// ****************************************************************************
struct Fault
{
    /// Simulated time of the event.
    Seconds at{};
    /// Name of the resource (a device of the robot).
    std::string resource;
    /// True disables the resource, false restores it.
    bool disable = true;
};

// ****************************************************************************
/// A resource failing at random, @c rate times per second on average.
///
/// @code
/// robotik::RandomFault const flaky{ "gripper", 0.05 }; // once every 20 s
/// @endcode
// ****************************************************************************
struct RandomFault
{
    /// Name of the resource.
    std::string resource;
    /// Mean number of failures per second.
    double rate = 0.0;
};

// ****************************************************************************
/// Applies a fault plan to a @ref ResourceManager over time.
///
/// The @ref Simulation builds it from the @c faults section of the scenario
/// and calls @ref update every step.
///
/// @code
/// robotik::FaultInjector faults({ { Seconds(3.0), "camera", true } }, {});
/// faults.reset(seed.derive("faults"));
/// while (running)
///     faults.update(resources, now, dt);
/// @endcode
// ****************************************************************************
class FaultInjector
{
public:

    FaultInjector() = default;

    // -------------------------------------------------------------------------
    /// @param p_scheduled Faults at fixed times, in any order.
    /// @param p_random Faults drawn at random.
    // -------------------------------------------------------------------------
    FaultInjector(std::vector<Fault> p_scheduled,
                  std::vector<RandomFault> p_random);

    // -------------------------------------------------------------------------
    /// Rewinds the plan to time zero (episode start).
    /// @param p_seed Drives the random faults: same seed, same failures.
    // -------------------------------------------------------------------------
    void reset(Seed p_seed);

    // -------------------------------------------------------------------------
    /// Applies the faults due in (@p_now - @p_dt, @p_now].
    /// @param p_resources Resources to fail or restore.
    /// @param p_now Simulated time at the end of the step.
    /// @param p_dt Duration of the step.
    // -------------------------------------------------------------------------
    void update(ResourceManager& p_resources, Seconds p_now, Seconds p_dt);

    // -------------------------------------------------------------------------
    /// @return The faults at fixed times.
    // -------------------------------------------------------------------------
    [[nodiscard]] std::vector<Fault> const& scheduled() const
    {
        return m_scheduled;
    }

    // -------------------------------------------------------------------------
    /// @return The faults drawn at random.
    // -------------------------------------------------------------------------
    [[nodiscard]] std::vector<RandomFault> const& random() const
    {
        return m_random_faults;
    }

private:

    /// Faults at fixed times, sorted by time.
    std::vector<Fault> m_scheduled;
    /// Faults drawn at random.
    std::vector<RandomFault> m_random_faults;
    /// Index of the next scheduled fault to apply.
    std::size_t m_next = 0;
    /// Generator of the random faults.
    Random m_random;
};

} // namespace robotik
