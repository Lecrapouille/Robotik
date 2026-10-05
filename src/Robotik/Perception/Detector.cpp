// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "Robotik/Perception/Detector.hpp"

#include <algorithm>
#include <cmath>

namespace robotik
{

Detections const& PerceptionPipeline::process(CameraFrame const& p_frame)
{
    m_detections.items.clear();
    m_detections.camera = p_frame.pose;
    m_detections.intrinsics = p_frame.intrinsics;
    m_detections.stamp = p_frame.stamp;
    m_detections.sequence = p_frame.sequence;
    for (std::unique_ptr<Detector> const& stage : m_stages)
    {
        stage->detect(p_frame, m_detections);
    }
    return m_detections;
}

void DepthEstimator::detect(CameraFrame const& p_frame,
                            Detections& p_detections)
{
    Image const& depth = p_frame.depth;
    if (depth.empty() || depth.format() != PixelFormat::Depth32F)
    {
        return;
    }

    int const width = static_cast<int>(depth.width());
    int const height = static_cast<int>(depth.height());
    for (Detection& detection : p_detections.items)
    {
        if (detection.position || detection.pose)
        {
            continue;
        }
        int const u = static_cast<int>(detection.center[0]);
        int const v = static_cast<int>(detection.center[1]);
        m_samples.clear();
        for (int y = std::max(0, v - m_radius);
             y <= std::min(height - 1, v + m_radius);
             ++y)
        {
            float const* row = depth.row<float>(static_cast<std::uint32_t>(y));
            for (int x = std::max(0, u - m_radius);
                 x <= std::min(width - 1, u + m_radius);
                 ++x)
            {
                if (std::isfinite(row[x]) && row[x] > 0.0f)
                {
                    m_samples.push_back(row[x]);
                }
            }
        }
        if (m_samples.empty())
        {
            continue;
        }
        auto const middle =
            m_samples.begin() + std::ptrdiff_t(m_samples.size() / 2u);
        std::nth_element(m_samples.begin(), middle, m_samples.end());
        Vector3 const ray =
            p_frame.intrinsics.ray(detection.center[0], detection.center[1]);
        // The depth is measured along the optical axis, not along the ray.
        detection.position = ray * (static_cast<double>(*middle) / ray.z);
    }
}

} // namespace robotik
