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
inline ImVec4 const GREY(0.59F, 0.62F, 0.66F, 1.0F);
inline ImVec4 const ORANGE(0.93F, 0.71F, 0.39F, 1.0F);
inline ImVec4 const GREEN(0.51F, 0.78F, 0.63F, 1.0F);
inline ImVec4 const RED(0.93F, 0.57F, 0.57F, 1.0F);
inline ImVec4 const BLUE(0.53F, 0.73F, 0.91F, 1.0F);
