// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file Localization.hpp
/// Robot pose in a map from fiducials of known placement.
///
/// Works with any detector that fills @ref robotik::Detection::id and
/// @ref robotik::Detection::pose (AprilTag, ArUco...). The chain is
/// camera frame -> robot frame -> map frame:
/// @code
/// map_T_base = map_T_tag * inverse(camera_T_tag) * inverse(base_T_camera)
/// @endcode
///
/// @par Contract
/// - Promises: no pose when no known marker is seen.
#pragma once

#include "Robotik/Task/Perception/Detection.hpp"

#include <optional>
#include <span>

namespace robotik
{

// ****************************************************************************
/// A fiducial of known placement in the map frame.
///
/// @code
/// robotik::Landmark tag{ .id = 7, .pose = Pose{ Vector3{ 2.0, 0.0, 0.5 } } };
/// @endcode
// ****************************************************************************
struct Landmark
{
    /// Fiducial id, matched to @ref Detection::id.
    int id = -1;
    /// Fiducial frame in the map (same convention as the detector poses).
    Pose pose;
};

// ****************************************************************************
/// Result of a localization.
///
/// @code
/// if (auto const where = robotik::localize(detections, landmarks))
///     std::cout << where->pose.position.x << " from " << where->landmarks << "
///     tags\n";
/// @endcode
// ****************************************************************************
struct Localization
{
    /// Robot base frame in the map frame.
    Pose pose;
    /// Number of landmarks averaged.
    std::size_t landmarks = 0;
};

// ----------------------------------------------------------------------------
/// Averages the robot pose given by every known fiducial of a frame.
///
/// Detections without a known id, or without a pose, are skipped. More tags
/// give a more stable result.
///
/// @param p_detections Detections of one camera frame, with their pose in
///   the robot base frame.
/// @param p_landmarks Known fiducials of the map.
/// @return The robot pose, or nothing when no detection matched a landmark.
// ----------------------------------------------------------------------------
[[nodiscard]] std::optional<Localization>
localize(Detections const& p_detections, std::span<Landmark const> p_landmarks);

} // namespace robotik
