// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

//! @file Detection.hpp
//! @brief What a detector saw in one image (not yet world knowledge).
//!
//! A detection lives in image space and, at best, in the camera frame. Turning
//! it into robot or world coordinates is the job of @ref WorldModel or of
//! @ref localize, which use the camera pose stored next to the detections.
#pragma once

#include "Robotik/Math/Geometry.hpp"
#include "Robotik/Sensors/Camera.hpp"

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace robotik
{

// ****************************************************************************
//! @brief One observation in an image.
// ****************************************************************************
struct Detection
{
    //!< Class or object name (e.g. "red_cube", "tag36h11").
    std::string label;
    //!< Instance id when the detector knows it (fiducial id), else -1.
    int id = -1;
    //!< Detector confidence in [0, 1].
    float confidence = 0.0f;
    //!< Bounding box in pixels, inclusive, top-left origin: x0, y0, x1, y1.
    std::array<int, 4> box{ 0, 0, 0, 0 };
    //!< Image point standing for the detection (centroid), in pixels.
    std::array<float, 2> center{ 0.0f, 0.0f };
    //!< 3D point in the optical frame (m), e.g. from depth.
    std::optional<Vector3> position;
    //!< Full pose in the optical frame (m), e.g. from a fiducial.
    std::optional<Pose> pose;
};

// ****************************************************************************
//! @brief Detections of one frame with the camera geometry needed to lift
//! them into the robot frame.
// ****************************************************************************
struct Detections
{
    std::vector<Detection> items;
    //!< Optical frame in the robot base frame at capture time.
    Pose camera;
    CameraIntrinsics intrinsics;
    Seconds stamp{};
    std::uint64_t sequence = 0;

    //! @brief First detection with @p_label, or null.
    [[nodiscard]] Detection const* find(std::string_view p_label) const
    {
        for (Detection const& detection : items)
        {
            if (detection.label == p_label)
            {
                return &detection;
            }
        }
        return nullptr;
    }
};

} // namespace robotik
