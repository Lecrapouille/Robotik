#include "Robotik/Perception/ColorDetector.hpp"

#include <algorithm>
#include <cmath>

namespace robotik
{

namespace
{

constexpr float kMaxHueDistance = 15.0f;
constexpr float kMinSaturation = 0.45f;
constexpr float kMinValue = 0.15f;
constexpr int kMinPixels = 20;

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

std::vector<ecs::Detection>
ColorDetector::detect(std::span<const std::uint8_t> p_rgb, int p_width, int p_height) const
{
    std::vector<ecs::Detection> found;
    if (p_rgb.size() < static_cast<std::size_t>(p_width * p_height * 3))
    {
        return found;
    }

    for (Target const& target : m_targets)
    {
        Hsv const want = hsv(target.color[0], target.color[1], target.color[2]);
        ecs::Detection box{ target.label, 0.0f, p_width, p_height, -1, -1 };
        int pixels = 0;
        for (int y = 0; y < p_height; ++y)
        {
            for (int x = 0; x < p_width; ++x)
            {
                std::uint8_t const* pixel = &p_rgb[static_cast<std::size_t>((y * p_width + x) * 3)];
                Hsv const have = hsv(pixel[0] / 255.0f, pixel[1] / 255.0f, pixel[2] / 255.0f);
                if (have.value < kMinValue || have.saturation < kMinSaturation ||
                    hueDistance(have.hue, want.hue) > kMaxHueDistance)
                {
                    continue;
                }
                int const row = p_height - 1 - y;
                box.x0 = std::min(box.x0, x);
                box.x1 = std::max(box.x1, x);
                box.y0 = std::min(box.y0, row);
                box.y1 = std::max(box.y1, row);
                ++pixels;
            }
        }
        if (pixels >= kMinPixels)
        {
            int const area = (box.x1 - box.x0 + 1) * (box.y1 - box.y0 + 1);
            box.confidence = static_cast<float>(pixels) / static_cast<float>(area);
            found.push_back(std::move(box));
        }
    }
    return found;
}

} // namespace robotik
