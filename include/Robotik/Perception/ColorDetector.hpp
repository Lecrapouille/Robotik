// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/**
 * @file ColorDetector.hpp
 * @brief Hue-based 2D object detection on RGB8 images (CPU, no OpenCV required).
 */

#pragma once

#include "Robotik/ECS/PerceptionComponents.hpp"

#include <array>
#include <cstdint>
#include <span>
#include <string>
#include <vector>

namespace robotik
{

/**
 * @brief Finds colored blobs by comparing pixel hue to registered targets.
 *
 * Output matches @ref ecs::Detection so a learned or OpenCV backend can
 * replace this class without changing ECS or skills.
 *
 * @example
 * @code
 * robotik::ColorDetector detector;
 * detector.add("red_cube", {0.9f, 0.1f, 0.1f});
 * auto boxes = detector.detect(rgb_span, width, height);
 * @endcode
 */
class ColorDetector
{
public:

    /** @brief Label and reference RGB color in @c [0, 1]. */
    struct Target
    {
        /** @brief Name stored in @ref ecs::Detection::label. */
        std::string label;

        /** @brief Reference color (R, G, B). */
        std::array<float, 3> color;
    };

    /**
     * @brief Registers one color class to detect.
     * @param p_label Object name.
     * @param p_color RGB in @c [0, 1].
     */
    void add(std::string p_label, std::array<float, 3> p_color)
    {
        m_targets.push_back({ std::move(p_label), p_color });
    }

    /**
     * @brief Runs detection on an RGB8 buffer.
     * @param p_rgb Three bytes per pixel; rows stored bottom-up (OpenGL order).
     * @param p_width Image width.
     * @param p_height Image height.
     * @return Bounding boxes with heuristic confidence.
     */
    [[nodiscard]] std::vector<ecs::Detection>
    detect(std::span<const std::uint8_t> p_rgb, int p_width, int p_height) const;

private:

    /** @brief Registered color targets. */
    std::vector<Target> m_targets;
};

} // namespace robotik
