// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#pragma once

#include <imgui.h>

// Status colors shared by every panel: idle, waiting, running, success, failure.
inline ImVec4 const GREY(0.55f, 0.55f, 0.58f, 1.0f);
inline ImVec4 const ORANGE(1.00f, 0.70f, 0.20f, 1.0f);
inline ImVec4 const GREEN(0.35f, 0.85f, 0.40f, 1.0f);
inline ImVec4 const RED(0.95f, 0.35f, 0.35f, 1.0f);
inline ImVec4 const BLUE(0.45f, 0.65f, 1.00f, 1.0f);
