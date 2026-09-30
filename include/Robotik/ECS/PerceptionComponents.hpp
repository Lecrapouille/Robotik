// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/**
 * @file PerceptionComponents.hpp
 * @brief On-robot camera metadata and 2D detection results in ECS.
 */

#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace robotik::ecs
{

/**
 * @brief Resolution and field of view for a link-mounted camera entity.
 *
 * Rendering and readback are done by the application; this library only
 * stores parameters and detection output.
 */
struct CameraSensor
{
    /** @brief Image width in pixels. */
    std::uint32_t width = 320;

    /** @brief Image height in pixels. */
    std::uint32_t height = 240;

    /** @brief Vertical field of view in degrees. */
    float fov_degrees = 70.0f;
};

/**
 * @brief Axis-aligned bounding box in image space.
 *
 * Origin is the top-left corner of the image; @c y increases downward.
 */
struct Detection
{
    /** @brief Class or object name (e.g. scenario object name). */
    std::string label;

    /** @brief Heuristic confidence in @c [0, 1]. */
    float confidence = 0.0f;

    /** @brief Left edge in pixels. */
    int x0 = 0;

    /** @brief Top edge in pixels. */
    int y0 = 0;

    /** @brief Right edge in pixels (inclusive). */
    int x1 = 0;

    /** @brief Bottom edge in pixels (inclusive). */
    int y1 = 0;
};

/**
 * @brief Latest detections written by the perception pipeline each frame.
 */
struct DetectedObjects
{
    /** @brief Detections from the most recent update. */
    std::vector<Detection> items;

    /** @brief Monotonic frame counter incremented by the application. */
    std::uint64_t frame = 0;
};

} // namespace robotik::ecs
