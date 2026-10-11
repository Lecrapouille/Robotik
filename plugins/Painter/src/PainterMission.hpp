// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#pragma once

#include "PainterScene.hpp"

#include "Robotik/Runtime/Mission.hpp"

#include "Compages/Core/Quaternion.hpp"
#include "Compages/World/Entity.hpp"

// Loads the floating easel and the free canvas, then the dip/mark skills.
class PainterMission final: public robotik::Mission
{
public:

    void setup(robotik::Simulation& p_simulation, robotik::SceneView* p_view) override;
    void reset(robotik::Simulation& p_simulation, robotik::Seed p_seed) override;
    void step(robotik::Simulation& p_simulation, robotik::Seconds p_dt) override;

private:

    void sync(robotik::Simulation& p_simulation);

    compages::core::Quatf m_up{};
    compages::world::Entity m_easel;
    compages::world::Entity m_canvas;
    robotik::SceneView* m_view = nullptr;
    painter::Stroke m_stroke;
    bool m_drawn = false;
};
