// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#pragma once

#include <imgui.h>

// Status colors shared by every panel: idle, waiting, running, success,
// failure.
inline ImVec4 const GREY(0.55F, 0.55F, 0.58F, 1.0F);
inline ImVec4 const ORANGE(1.00F, 0.70F, 0.20F, 1.0F);
inline ImVec4 const GREEN(0.35F, 0.85F, 0.40F, 1.0F);
inline ImVec4 const RED(0.95F, 0.35F, 0.35F, 1.0F);
inline ImVec4 const BLUE(0.45F, 0.65F, 1.00F, 1.0F);
