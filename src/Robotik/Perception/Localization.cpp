// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "Robotik/Perception/Localization.hpp"

namespace robotik
{

std::optional<Localization>
localize(Detections const& p_detections, std::span<Landmark const> p_landmarks)
{
    Pose const camera_in_base_inverse = p_detections.camera.inverse();
    Vector3 position;
    Quaternion sum{ 0.0, 0.0, 0.0, 0.0 };
    Quaternion reference;
    std::size_t count = 0;

    for (Detection const& detection : p_detections.items)
    {
        if (!detection.pose || detection.id < 0)
        {
            continue;
        }
        for (Landmark const& landmark : p_landmarks)
        {
            if (landmark.id != detection.id)
            {
                continue;
            }
            Pose const base = landmark.pose * detection.pose->inverse() *
                              camera_in_base_inverse;
            // q and -q are the same rotation: align before summing.
            Quaternion q = base.rotation;
            if (count == 0u)
            {
                reference = q;
            }
            else if (reference.w * q.w + reference.x * q.x +
                         reference.y * q.y + reference.z * q.z <
                     0.0)
            {
                q = { -q.w, -q.x, -q.y, -q.z };
            }
            sum = { sum.w + q.w, sum.x + q.x, sum.y + q.y, sum.z + q.z };
            position += base.position;
            ++count;
            break;
        }
    }

    if (count == 0u)
    {
        return std::nullopt;
    }
    double const scale = 1.0 / static_cast<double>(count);
    return Localization{ Pose{ position * scale, sum.normalized() }, count };
}

} // namespace robotik
