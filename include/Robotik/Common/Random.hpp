// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file Random.hpp
/// Deterministic seeds and a tiny random generator.
///
/// There is no global generator: every consumer (world layout, sensor noise,
/// faults, each RL environment) derives its own @ref Seed from a master seed,
/// so adding a consumer never shifts the random stream of another one.
///
/// @code
/// robotik::Seed const master(123456);
/// robotik::Random world(master.derive("world"));
/// robotik::Seed const env3 = master.derive(3u);
/// robotik::Seed const episode = env3.derive(42u);
/// @endcode
#pragma once

#include <cmath>
#include <cstdint>
#include <numbers>
#include <string_view>

namespace robotik
{

// -------------------------------------------------------------------------
/// SplitMix64 finalizer: a strong 64-bit mixing function. Close inputs give
/// very different outputs.
/// @param p_value Value to mix.
/// @return The mixed value.
// -------------------------------------------------------------------------
[[nodiscard]] constexpr std::uint64_t mix64(std::uint64_t p_value)
{
    p_value += 0x9E3779B97F4A7C15ULL;
    p_value = (p_value ^ (p_value >> 30)) * 0xBF58476D1CE4E5B9ULL;
    p_value = (p_value ^ (p_value >> 27)) * 0x94D049BB133111EBULL;
    return p_value ^ (p_value >> 31);
}

// ****************************************************************************
/// Node of the seed hierarchy: a value plus deterministic children.
///
/// A seed is not a generator: it only names a random stream. Give it to a
/// @ref Random to draw numbers. The same parent and the same name always give
/// the same child, on every machine.
///
/// @code
/// robotik::Seed const master(7);
/// robotik::Seed const noise = master.derive("sensors").derive("camera");
/// robotik::Random rng(noise);
/// @endcode
// ****************************************************************************
struct Seed
{
    /// Raw 64-bit seed.
    std::uint64_t value = 0;

    constexpr Seed() = default;

    /// @param p_value Raw seed, for example from the command line.
    constexpr explicit Seed(std::uint64_t p_value) : value(p_value) {}

    // -------------------------------------------------------------------------
    /// Child seed for an indexed consumer (environment, episode...).
    /// @param p_index Index of the consumer.
    /// @return A seed independent of the other indices.
    // -------------------------------------------------------------------------
    [[nodiscard]] constexpr Seed derive(std::uint64_t p_index) const
    {
        return Seed(mix64(value ^ mix64(p_index + 0x632BE59BD9B4E019ULL)));
    }

    // -------------------------------------------------------------------------
    /// Child seed for a named consumer ("world", "sensors"...).
    /// @param p_name Name of the consumer, hashed with FNV-1a.
    /// @return A seed independent of the other names.
    // -------------------------------------------------------------------------
    [[nodiscard]] constexpr Seed derive(std::string_view p_name) const
    {
        std::uint64_t hash = 0xCBF29CE484222325ULL; // FNV-1a
        for (char const c : p_name)
        {
            hash = (hash ^ static_cast<std::uint8_t>(c)) * 0x100000001B3ULL;
        }
        return derive(hash);
    }

    /// Same value, same seed.
    [[nodiscard]] constexpr bool operator==(Seed const&) const = default;
};

// ****************************************************************************
/// SplitMix64 generator: 8 bytes of state, fast and reproducible on every
/// platform (unlike the std distributions).
///
/// @code
/// robotik::Random rng(seed);
/// double x = rng.uniform(-0.1, 0.1);   // position jitter
/// double n = rng.normal(0.0, 0.01);    // sensor noise
/// if (rng.chance(0.05)) { ... }        // rare event
/// @endcode
// ****************************************************************************
class Random
{
public:

    /// @param p_seed Start of the stream: same seed, same numbers.
    constexpr explicit Random(Seed p_seed = Seed{}) noexcept
        : m_state(p_seed.value)
    {
    }

    // -------------------------------------------------------------------------
    /// Draws the next raw value.
    /// @return A uniform 64-bit value.
    // -------------------------------------------------------------------------
    constexpr std::uint64_t next()
    {
        m_state += 0x9E3779B97F4A7C15ULL;
        std::uint64_t z = m_state;
        z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
        z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
        return z ^ (z >> 31);
    }

    // -------------------------------------------------------------------------
    /// Draws a uniform number between 0 and 1.
    /// @return A double in [0, 1).
    // -------------------------------------------------------------------------
    constexpr double uniform()
    {
        return static_cast<double>(next() >> 11) * 0x1.0p-53;
    }

    // -------------------------------------------------------------------------
    /// Draws a uniform number in a range.
    /// @param p_low Lowest value (included).
    /// @param p_high Highest value (excluded).
    /// @return A double in [p_low, p_high).
    // -------------------------------------------------------------------------
    constexpr double uniform(double p_low, double p_high)
    {
        return p_low + (p_high - p_low) * uniform();
    }

    // -------------------------------------------------------------------------
    /// Draws a Gaussian number (Box-Muller). There is no cached second
    /// value, so the stream only depends on the number of calls.
    /// @param p_mean Mean of the distribution.
    /// @param p_sigma Standard deviation.
    /// @return The sample.
    // -------------------------------------------------------------------------
    double normal(double p_mean = 0.0, double p_sigma = 1.0)
    {
        double const u = 1.0 - uniform();
        double const v = uniform();
        return p_mean + p_sigma * std::sqrt(-2.0 * std::log(u)) *
                            std::cos(2.0 * std::numbers::pi * v);
    }

    // -------------------------------------------------------------------------
    /// Draws a yes/no event.
    /// @param p_probability Probability of true, in [0, 1].
    /// @return True with probability @p_probability.
    // -------------------------------------------------------------------------
    constexpr bool chance(double p_probability)
    {
        return uniform() < p_probability;
    }

private:

    /// State of the generator.
    std::uint64_t m_state;
};

} // namespace robotik
