// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#pragma once

#include "Compages/Core/Units.hpp"

struct App;
namespace compages::renderer
{
class RenderTarget;
}
struct Window;

namespace compages::world
{
struct ViewFrame;
}

/// Mouse input for the world panel, in panel pixels with Y up.
/// @param p_app The application.
/// @param p_elapsed The elapsed time.
/// @param p_total The total time.
/// @return The view frame.
/// Where a 3D view is drawn on the screen (ImGui pixels, top-left corner and
/// size), and whether the mouse is on it.
struct ViewArea
{
    float x = 0.0F;
    float y = 0.0F;
    float w = 1.0F;
    float h = 1.0F;
    bool hovered = false;
};

/// Input and size of the 3D view drawn into @p_target at @p_area: what a
/// camera rig of its world reads.
/// @p_simulated: time of the simulation since the previous picture (zero
/// while paused), seen by the routines; @p_wall: time of the wall clock, seen
/// by the camera rig; @p_total: time of the simulation.
compages::world::ViewFrame viewFrame(compages::renderer::RenderTarget const& p_target,
                                     ViewArea const& p_area,
                                     Window const& p_window,
                                     compages::si::Seconds p_simulated,
                                     compages::si::Seconds p_wall,
                                     compages::si::Seconds p_total);

/// The same for the 3D view of the scenario.
compages::world::ViewFrame viewFrame(App const& p_app,
                                     Window const& p_window,
                                     compages::si::Seconds p_simulated,
                                     compages::si::Seconds p_wall,
                                     compages::si::Seconds p_total);
