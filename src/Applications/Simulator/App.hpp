// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#pragma once

#include "ColorDetector.hpp"
#include "SimulatorView.hpp"

#include "Robotik/Runtime/Simulation.hpp"

#include "Compages/Renderer/Scene.hpp"
#include "Compages/World/World.hpp"

#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>

// Everything the operator interface shows and drives. The whole world is
// rebuilt from the scenario file on load; reset replays it with a seed.
struct App
{
    std::filesystem::path scenario_path;
    std::string error;

    std::unique_ptr<compages::world::World> world;
    std::unique_ptr<compages::renderer::Scene> scene;
    std::unique_ptr<SimulatorView> scene_view;
    std::unique_ptr<robotik::Simulation> simulation;
    ColorDetector* detector = nullptr;
    compages::world::Entity view_camera;

    RenderTarget view;
    bool view_hovered = false;

    bool playing = true;
    bool step_once = false;
    float speed = 1.0f;
    std::uint64_t seed = 0;

    void load();
    void reset(std::uint64_t p_seed);
    // Advances the simulation by fixed steps, as the headless runner does.
    void advance(double p_elapsed);

private:

    double m_lag = 0.0;
    bool m_reported = false;
};

// The dock layout and every panel, drawn each frame.
void drawPanels(App& p_app);
