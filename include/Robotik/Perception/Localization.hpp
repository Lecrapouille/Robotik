// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

//! @file Localization.hpp
//! @brief Robot pose in a map from fiducials of known placement.
//!
//! Works with any detector that fills @ref Detection::id and
//! @ref Detection::pose (AprilTag, ArUco...). The chain is
//! camera frame -> robot frame -> map frame:
//! @code
//! map_T_base = map_T_tag * inverse(camera_T_tag) * inverse(base_T_camera)
//! @endcode
#pragma once

#include "Robotik/Perception/Detection.hpp"

#include <optional>
#include <span>

namespace robotik
{

// ****************************************************************************
//! @brief A fiducial of known placement in the map frame.
// ****************************************************************************
struct Landmark
{
    int id = -1;
    //!< Fiducial frame in the map (same convention as the detector poses).
    Pose pose;
};

// ****************************************************************************
//! @brief Result of a localization.
// ****************************************************************************
struct Localization
{
    //!< Robot base frame in the map frame.
    Pose pose;
    //!< Number of landmarks averaged.
    std::size_t landmarks = 0;
};

// ----------------------------------------------------------------------------
//! @brief Averages the robot pose given by every known fiducial of a frame.
//! @return Nothing when no detection matched a landmark.
// ----------------------------------------------------------------------------
[[nodiscard]] std::optional<Localization>
localize(Detections const& p_detections, std::span<Landmark const> p_landmarks);

} // namespace robotik
