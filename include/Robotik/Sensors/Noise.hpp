// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

//! @file Noise.hpp
//! @brief How a sensor degrades the true value: noise, bias, resolution,
//! saturation and latency.
//!
//! The model is applied in this order: bias, white gaussian noise,
//! quantization to the resolution, then saturation. An ideal channel (all
//! zero, unbounded) returns the true value and draws no random number, so
//! turning noise off does not shift the random streams of the other sensors.
#pragma once

#include "Robotik/Math/Geometry.hpp"
#include "Robotik/Math/Random.hpp"

#include <cmath>
#include <deque>
#include <limits>
#include <utility>

namespace robotik
{

// ****************************************************************************
//! @brief Degradation of one measured channel, in the unit of the channel.
// ****************************************************************************
struct NoiseModel
{
    //!< Standard deviation of the white gaussian noise.
    double sigma = 0.0;
    //!< Constant offset added to the true value.
    double bias = 0.0;
    //!< Quantization step; zero for a continuous output.
    double resolution = 0.0;
    //!< Saturation bounds of the output.
    double min = -std::numeric_limits<double>::infinity();
    double max = std::numeric_limits<double>::infinity();

    //! @brief Pure gaussian noise of standard deviation @p_sigma.
    [[nodiscard]] static NoiseModel gaussian(double p_sigma)
    {
        NoiseModel model;
        model.sigma = p_sigma;
        return model;
    }

    [[nodiscard]] bool ideal() const
    {
        return sigma <= 0.0 && std::fpclassify(bias) == FP_ZERO && resolution <= 0.0 &&
               std::isinf(min) && std::isinf(max);
    }

    [[nodiscard]] double apply(double p_value, Random& p_random) const
    {
        double value = p_value + bias;
        if (sigma > 0.0)
        {
            value = p_random.normal(value, sigma);
        }
        if (resolution > 0.0)
        {
            value = resolution * std::round(value / resolution);
        }
        return value < min ? min : (value > max ? max : value);
    }

    //! @brief Same model on the three axes (drawn x, then y, then z).
    [[nodiscard]] Vector3 apply(Vector3 const& p_value, Random& p_random) const
    {
        double const x = apply(p_value.x, p_random);
        double const y = apply(p_value.y, p_random);
        double const z = apply(p_value.z, p_random);
        return { x, y, z };
    }
};

// ****************************************************************************
//! @brief Delivers samples @c latency after their capture.
//!
//! The stamp of a sample stays its capture time: a consumer sees an old
//! measurement, and knows how old it is.
// ****************************************************************************
template <class Sample>
class LatencyBuffer
{
public:

    void latency(Seconds p_latency)
    {
        m_latency = p_latency;
    }

    [[nodiscard]] Seconds latency() const
    {
        return m_latency;
    }

    //! @brief Queues a sample captured at @p_now.
    void push(Seconds p_now, Sample p_sample)
    {
        m_pending.emplace_back(p_now + m_latency, std::move(p_sample));
    }

    //! @brief Moves the samples due at @p_now to @ref current. A nanosecond
    //! of slack absorbs the rounding of clocks summed step by step.
    //! @return True when a sample was delivered.
    bool advance(Seconds p_now)
    {
        bool delivered = false;
        while (!m_pending.empty() && m_pending.front().first.value() <= p_now.value() + 1e-9)
        {
            m_current = std::move(m_pending.front().second);
            m_pending.pop_front();
            delivered = true;
        }
        return delivered;
    }

    //! @brief Last delivered sample (default before the first one).
    [[nodiscard]] Sample const& current() const
    {
        return m_current;
    }

    [[nodiscard]] Sample& current()
    {
        return m_current;
    }

    void clear()
    {
        m_pending.clear();
        m_current = Sample{};
    }

private:

    Seconds m_latency{ 0.0 };
    std::deque<std::pair<Seconds, Sample>> m_pending;
    Sample m_current{};
};

} // namespace robotik
