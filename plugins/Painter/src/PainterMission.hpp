// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#pragma once

#include "PainterScene.hpp"

#include "Robotik/Runtime/Mission.hpp"

// The dip, mark and rinse gestures, and the ink left on the rendered canvas.
// The easel and the canvas belong to the world, not to the mission.
class PainterMission final: public robotik::Mission
{
public:

    void setup(robotik::Simulation& p_simulation, robotik::SceneView* p_view) override;
    void step(robotik::Simulation& p_simulation, robotik::Seconds p_dt) override;

private:

    robotik::SceneView* m_view = nullptr;
    painter::Stroke m_stroke;
};
