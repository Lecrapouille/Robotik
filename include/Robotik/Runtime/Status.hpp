// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

// @file Status.hpp
// @brief Execution status shared by skills and behavior-tree adapters.
#pragma once

namespace robotik
{

// -------------------------------------------------------------------------
// @brief Outcome of a single skill tick or an aggregated skill run.
// -------------------------------------------------------------------------
enum class Status
{
    IDLE,     //!< Not started or explicitly idle.
    RUNNING,  //!< Still working toward the goal.
    SUCCESS,  //!< Goal reached.
    FAILURE,  //!< Goal cannot be reached or was aborted.
};

} // namespace robotik
