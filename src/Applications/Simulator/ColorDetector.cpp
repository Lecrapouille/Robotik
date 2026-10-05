// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "ColorDetector.hpp"

#include <algorithm>
#include <cmath>

#define COLOR_MAX_HUE_DISTANCE 15.0f
#define COLOR_MIN_SATURATION 0.45f
#define COLOR_MIN_VALUE 0.15f
#define COLOR_MIN_PIXELS 20

namespace
{

struct Hsv
{
    float hue;
    float saturation;
    float value;
};

// Components in [0, 1], hue in degrees.
Hsv hsv(float p_r, float p_g, float p_b)
{
    float const high = std::max({ p_r, p_g, p_b });
    float const low = std::min({ p_r, p_g, p_b });
    float const range = high - low;
    Hsv result{ 0.0f, high > 0.0f ? range / high : 0.0f, high };
    if (range <= 0.0f)
    {
        return result;
    }
    if (high == p_r)
    {
        result.hue = 60.0f * std::fmod((p_g - p_b) / range + 6.0f, 6.0f);
    }
    else if (high == p_g)
    {
        result.hue = 60.0f * ((p_b - p_r) / range + 2.0f);
    }
    else
    {
        result.hue = 60.0f * ((p_r - p_g) / range + 4.0f);
    }
    return result;
}

float hueDistance(float p_a, float p_b)
{
    float const distance = std::abs(p_a - p_b);
    return std::min(distance, 360.0f - distance);
}

} // namespace

void ColorDetector::detect(robotik::CameraFrame const& p_frame,
                           robotik::Detections& p_detections)
{
    robotik::Image const& image = p_frame.rgb;
    if (image.format() != robotik::PixelFormat::RGB8 || image.empty())
    {
        return;
    }
    std::uint32_t const width = image.width();
    std::uint32_t const height = image.height();
    std::uint32_t const count = width * height;
    m_mask.resize(count);

    for (Target const& target : m_targets)
    {
        Hsv const want = hsv(target.color[0], target.color[1], target.color[2]);
        for (std::uint32_t y = 0; y < height; ++y)
        {
            std::uint8_t const* pixel = image.row<std::uint8_t>(y);
            std::uint8_t* mask = m_mask.data() + y * width;
            for (std::uint32_t x = 0; x < width; ++x, pixel += 3)
            {
                Hsv const have = hsv(pixel[0] / 255.0f, pixel[1] / 255.0f, pixel[2] / 255.0f);
                mask[x] = have.value >= COLOR_MIN_VALUE &&
                          have.saturation >= COLOR_MIN_SATURATION &&
                          hueDistance(have.hue, want.hue) <= COLOR_MAX_HUE_DISTANCE;
            }
        }

        // Largest 4-connected region; visited pixels are cleared from the mask.
        robotik::Detection best;
        std::uint32_t best_pixels = 0;
        for (std::uint32_t seed = 0; seed < count; ++seed)
        {
            if (m_mask[seed] == 0)
            {
                continue;
            }
            int x0 = static_cast<int>(width);
            int y0 = static_cast<int>(height);
            int x1 = -1;
            int y1 = -1;
            double sum_x = 0.0;
            double sum_y = 0.0;
            std::uint32_t pixels = 0;
            m_mask[seed] = 0;
            m_stack.assign(1, seed);
            while (!m_stack.empty())
            {
                std::uint32_t const at = m_stack.back();
                m_stack.pop_back();
                int const x = static_cast<int>(at % width);
                int const y = static_cast<int>(at / width);
                x0 = std::min(x0, x);
                x1 = std::max(x1, x);
                y0 = std::min(y0, y);
                y1 = std::max(y1, y);
                sum_x += x;
                sum_y += y;
                ++pixels;
                auto visit = [&](std::uint32_t p_next)
                {
                    if (m_mask[p_next] != 0)
                    {
                        m_mask[p_next] = 0;
                        m_stack.push_back(p_next);
                    }
                };
                if (x > 0) visit(at - 1u);
                if (x + 1 < static_cast<int>(width)) visit(at + 1u);
                if (y > 0) visit(at - width);
                if (y + 1 < static_cast<int>(height)) visit(at + width);
            }
            if (pixels <= best_pixels)
            {
                continue;
            }
            best_pixels = pixels;
            best.box = { x0, y0, x1, y1 };
            best.center = { static_cast<float>(sum_x / pixels),
                            static_cast<float>(sum_y / pixels) };
            int const area = (x1 - x0 + 1) * (y1 - y0 + 1);
            best.confidence = static_cast<float>(pixels) / static_cast<float>(area);
        }
        if (best_pixels < COLOR_MIN_PIXELS)
        {
            continue;
        }
        best.label = target.label;
        p_detections.items.push_back(std::move(best));
    }
}
