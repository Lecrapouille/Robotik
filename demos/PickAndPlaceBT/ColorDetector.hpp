// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#pragma once

#include "Robotik/Perception/Detector.hpp"

#include <array>
#include <cstdint>
#include <string>
#include <vector>

// Hue-based blob detector on RGB8 images: the largest connected region of
// each target color. One of the interchangeable perception stages: a learned
// detector would implement the same interface.
class ColorDetector final: public robotik::Detector
{
public:

    struct Target
    {
        std::string label;
        // Reference color, RGB in [0, 1].
        std::array<float, 3> color;
    };

    void add(std::string p_label, std::array<float, 3> p_color)
    {
        m_targets.push_back({ std::move(p_label), p_color });
    }

    void detect(robotik::CameraFrame const& p_frame,
                robotik::Detections& p_detections) override;

private:

    std::vector<Target> m_targets;
    std::vector<std::uint8_t> m_mask;
    std::vector<std::uint32_t> m_stack;
};
