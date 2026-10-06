// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#pragma once

#include "Robotik/Runtime/Mission.hpp"

class ColorDetector;

// Pick-and-place task skills (Detect / Approach / Reach / Grasp / Release)
// and, when a camera is rendered, the color detector of the application.
class PickPlaceMission final: public robotik::Mission
{
public:

    // @p_skills false keeps only the detector (RL overlay).
    explicit PickPlaceMission(bool p_skills = true) : m_skills(p_skills) {}

    void setup(robotik::Simulation& p_simulation,
               robotik::SceneView* p_view) override;

    [[nodiscard]] ColorDetector* detector() const
    {
        return m_detector;
    }

private:

    bool m_skills = true;
    ColorDetector* m_detector = nullptr;
};
