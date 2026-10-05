// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

//! @file Random.hpp
//! @brief Deterministic seeds and a tiny random generator.
//!
//! There is no global generator: every consumer (world layout, sensor noise,
//! faults, each RL environment) derives its own @ref Seed from a master seed,
//! so adding a consumer never shifts the random stream of another one.
//!
//! @code
//! robotik::Seed const master(123456);
//! robotik::Random world(master.derive("world"));
//! robotik::Seed const env3 = master.derive(3u);
//! robotik::Seed const episode = env3.derive(42u);
//! @endcode
#pragma once

#include <cmath>
#include <cstdint>
#include <numbers>
#include <string_view>

namespace robotik
{

// -------------------------------------------------------------------------
//! @brief SplitMix64 finalizer: a strong 64-bit mixing function.
// -------------------------------------------------------------------------
[[nodiscard]] constexpr std::uint64_t mix64(std::uint64_t p_value)
{
    p_value += 0x9E3779B97F4A7C15ull;
    p_value = (p_value ^ (p_value >> 30)) * 0xBF58476D1CE4E5B9ull;
    p_value = (p_value ^ (p_value >> 27)) * 0x94D049BB133111EBull;
    return p_value ^ (p_value >> 31);
}

// ****************************************************************************
//! @brief Node of the seed hierarchy: a value plus deterministic children.
// ****************************************************************************
struct Seed
{
    std::uint64_t value = 0;

    constexpr Seed() = default;
    constexpr explicit Seed(std::uint64_t p_value) : value(p_value) {}

    // -------------------------------------------------------------------------
    //! @brief Child seed for an indexed consumer (environment, episode...).
    // -------------------------------------------------------------------------
    [[nodiscard]] constexpr Seed derive(std::uint64_t p_index) const
    {
        return Seed(mix64(value ^ mix64(p_index + 0x632BE59BD9B4E019ull)));
    }

    // -------------------------------------------------------------------------
    //! @brief Child seed for a named consumer ("world", "sensors"...).
    // -------------------------------------------------------------------------
    [[nodiscard]] constexpr Seed derive(std::string_view p_name) const
    {
        std::uint64_t hash = 0xCBF29CE484222325ull; // FNV-1a
        for (char const c : p_name)
        {
            hash = (hash ^ static_cast<std::uint8_t>(c)) * 0x100000001B3ull;
        }
        return derive(hash);
    }

    [[nodiscard]] constexpr bool operator==(Seed const&) const = default;
};

// ****************************************************************************
//! @brief SplitMix64 generator: 8 bytes of state, fast and reproducible on
//! every platform (unlike the std distributions).
// ****************************************************************************
class Random
{
public:

    constexpr explicit Random(Seed p_seed = Seed{}) noexcept : m_state(p_seed.value) {}

    // -------------------------------------------------------------------------
    //! @brief Next raw 64-bit value.
    // -------------------------------------------------------------------------
    constexpr std::uint64_t next()
    {
        m_state += 0x9E3779B97F4A7C15ull;
        std::uint64_t z = m_state;
        z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
        z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
        return z ^ (z >> 31);
    }

    // -------------------------------------------------------------------------
    //! @brief Uniform double in [0, 1).
    // -------------------------------------------------------------------------
    constexpr double uniform()
    {
        return static_cast<double>(next() >> 11) * 0x1.0p-53;
    }

    // -------------------------------------------------------------------------
    //! @brief Uniform double in [p_low, p_high).
    // -------------------------------------------------------------------------
    constexpr double uniform(double p_low, double p_high)
    {
        return p_low + (p_high - p_low) * uniform();
    }

    // -------------------------------------------------------------------------
    //! @brief Gaussian sample (Box-Muller, no cached second value so that the
    //! stream only depends on the number of calls).
    // -------------------------------------------------------------------------
    double normal(double p_mean = 0.0, double p_sigma = 1.0)
    {
        double const u = 1.0 - uniform();
        double const v = uniform();
        return p_mean + p_sigma * std::sqrt(-2.0 * std::log(u)) *
                            std::cos(2.0 * std::numbers::pi * v);
    }

    // -------------------------------------------------------------------------
    //! @brief True with probability @p_probability.
    // -------------------------------------------------------------------------
    constexpr bool chance(double p_probability)
    {
        return uniform() < p_probability;
    }

private:

    std::uint64_t m_state;
};

} // namespace robotik
