// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#pragma once

#include "PainterScene.hpp"

#include "Robotik/Task/Mission.hpp"

#include <map>
#include <string>
#include <vector>

// The dip, mark and rinse gestures, and the ink left on the rendered canvas.
// The easel and the canvas belong to the world, not to the mission.
class PainterMission final: public robotik::Mission
{
public:

    /// @p_planner links the PDDL domain (painter.domain.pddl) to the gestures.
    explicit PainterMission(bool p_planner = false) : m_planner(p_planner) {}

    void setup(robotik::Simulation& p_simulation,
               robotik::SceneView* p_view) override;
    void reset(robotik::Simulation& p_simulation,
               robotik::Seed p_seed) override;
    void step(robotik::Simulation& p_simulation,
              robotik::Seconds p_dt) override;
    /// @brief painter.stroke_deviation: largest distance (m), in the plane of
    /// the canvas, between the brush while it inks and the straight line of
    /// its stroke. painter.inked: number of steps that laid ink.
    void measure(robotik::Simulation const& p_simulation,
                 robotik::Metrics& p_metrics) const override;

private:

    void registerPlanning(robotik::Simulation& p_simulation);

private:

    bool m_planner = false;
    robotik::SceneView* m_view = nullptr;
    painter::Stroke m_stroke;
    /// Waypoints of each stroke, in the world frame.
    std::map<std::string, std::vector<robotik::Pose>> m_paths;
    double m_deviation = 0.0;
    int m_inked = 0;
};
