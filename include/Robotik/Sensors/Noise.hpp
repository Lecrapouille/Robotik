// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file Noise.hpp
/// How a sensor degrades the true value: noise, bias, resolution, saturation
/// and latency.
///
/// The model is applied in this order: bias, white gaussian noise,
/// quantization to the resolution, then saturation. An ideal channel (all
/// zero, unbounded) returns the true value and draws no random number, so
/// turning noise off does not shift the random streams of the other sensors.
#pragma once

#include "Robotik/Common/Math.hpp"
#include "Robotik/Common/Random.hpp"

#include <cmath>
#include <deque>
#include <limits>
#include <utility>

namespace robotik
{

// ****************************************************************************
/// Degradation of one measured channel, in the unit of the channel.
///
/// @code
/// robotik::NoiseModel range = robotik::NoiseModel::gaussian(0.01); // 1 cm
/// range.resolution = 0.001;                                       // 1 mm
/// steps range.min = 0.0; range.max = 10.0; double const measured =
/// range.apply(true_distance, rng);
/// @endcode
// ****************************************************************************
struct NoiseModel
{
    /// Standard deviation of the white gaussian noise.
    double sigma = 0.0;
    /// Constant offset added to the true value.
    double bias = 0.0;
    /// Quantization step; zero for a continuous output.
    double resolution = 0.0;
    /// Lowest output (saturation).
    double min = -std::numeric_limits<double>::infinity();
    /// Highest output (saturation).
    double max = std::numeric_limits<double>::infinity();

    // -------------------------------------------------------------------------
    /// Builds a pure gaussian noise.
    /// @param p_sigma Standard deviation.
    /// @return A model without bias, quantization nor saturation.
    // -------------------------------------------------------------------------
    [[nodiscard]] static NoiseModel gaussian(double p_sigma)
    {
        NoiseModel model;
        model.sigma = p_sigma;
        return model;
    }

    // -------------------------------------------------------------------------
    /// @return True when the model changes nothing (and draws nothing).
    // -------------------------------------------------------------------------
    [[nodiscard]] bool ideal() const
    {
        return sigma <= 0.0 && std::fpclassify(bias) == FP_ZERO &&
               resolution <= 0.0 && std::isinf(min) && std::isinf(max);
    }

    // -------------------------------------------------------------------------
    /// Degrades one value.
    /// @param p_value True value.
    /// @param p_random Generator of the sensor.
    /// @return The measured value.
    // -------------------------------------------------------------------------
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

    // -------------------------------------------------------------------------
    /// Degrades a vector with the same model on the three axes (drawn x,
    /// then y, then z).
    /// @param p_value True vector.
    /// @param p_random Generator of the sensor.
    /// @return The measured vector.
    // -------------------------------------------------------------------------
    [[nodiscard]] Vector3 apply(Vector3 const& p_value, Random& p_random) const
    {
        double const x = apply(p_value.x, p_random);
        double const y = apply(p_value.y, p_random);
        double const z = apply(p_value.z, p_random);
        return { x, y, z };
    }
};

// ****************************************************************************
/// Delivers samples @c latency after their capture.
///
/// The stamp of a sample stays its capture time: a consumer sees an old
/// measurement, and knows how old it is.
///
/// @code
/// robotik::LatencyBuffer<robotik::ImuReading> buffer;
/// buffer.latency(Seconds(0.02));
/// buffer.push(now, reading);           // captured now
/// if (buffer.advance(now + Seconds(0.02)))
///     use(buffer.current());           // delivered 20 ms later
/// @endcode
///
/// @tparam Sample Reading type, default constructible.
// ****************************************************************************
template <class Sample>
class LatencyBuffer
{
public:

    // -------------------------------------------------------------------------
    /// Sets the delay between a capture and its delivery.
    /// @param p_latency Delay; zero delivers at once.
    // -------------------------------------------------------------------------
    void latency(Seconds p_latency)
    {
        m_latency = p_latency;
    }

    // -------------------------------------------------------------------------
    /// @return The delay between a capture and its delivery.
    // -------------------------------------------------------------------------
    [[nodiscard]] Seconds latency() const
    {
        return m_latency;
    }

    // -------------------------------------------------------------------------
    /// Queues a sample.
    /// @param p_now Capture time.
    /// @param p_sample Captured sample.
    // -------------------------------------------------------------------------
    void push(Seconds p_now, Sample p_sample)
    {
        m_pending.emplace_back(p_now + m_latency, std::move(p_sample));
    }

    // -------------------------------------------------------------------------
    /// Moves the samples due at @p_now to @ref current. A nanosecond of
    /// slack absorbs the rounding of clocks summed step by step.
    /// @param p_now Current time.
    /// @return True when a sample was delivered.
    // -------------------------------------------------------------------------
    bool advance(Seconds p_now)
    {
        bool delivered = false;
        while (!m_pending.empty() &&
               m_pending.front().first.value() <= p_now.value() + 1e-9)
        {
            m_current = std::move(m_pending.front().second);
            m_pending.pop_front();
            delivered = true;
        }
        return delivered;
    }

    // -------------------------------------------------------------------------
    /// @return Last delivered sample (default before the first one).
    // -------------------------------------------------------------------------
    [[nodiscard]] Sample const& current() const
    {
        return m_current;
    }

    /// @copydoc current() const
    [[nodiscard]] Sample& current()
    {
        return m_current;
    }

    // -------------------------------------------------------------------------
    /// Drops the pending samples and resets @ref current (episode reset).
    // -------------------------------------------------------------------------
    void clear()
    {
        m_pending.clear();
        m_current = Sample{};
    }

private:

    /// Delay between a capture and its delivery.
    Seconds m_latency{ 0.0 };
    /// Samples not yet delivered, with their delivery time.
    std::deque<std::pair<Seconds, Sample>> m_pending;
    /// Last delivered sample.
    Sample m_current{};
};

} // namespace robotik
