// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#pragma once

struct App;

namespace compages::world
{
struct ViewFrame;
}

//! @brief Mouse input for the world panel, in panel pixels with Y up.
//! @param p_app The application.
//! @param p_elapsed The elapsed time.
//! @param p_total The total time.
//! @return The view frame.
compages::world::ViewFrame
viewFrame(App const& p_app, float p_elapsed, float p_total);
