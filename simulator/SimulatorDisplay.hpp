// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#pragma once

#include "Compages/Core/Vector.hpp"

#define VIEW_CLEAR_R 0.12F
#define VIEW_CLEAR_G 0.14F
#define VIEW_CLEAR_B 0.18F
#define VIEW_CLEAR_A 1.0F

inline compages::core::Vector4f viewClearColor()
{
    return { VIEW_CLEAR_R, VIEW_CLEAR_G, VIEW_CLEAR_B, VIEW_CLEAR_A };
}
