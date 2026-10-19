// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file Status.hpp
/// Execution status shared by skills and behavior-tree adapters.
#pragma once

namespace robotik
{

// -------------------------------------------------------------------------
/// Outcome of a single skill tick or an aggregated skill run.
///
/// A skill returns @c RUNNING while it works, then @c SUCCESS or @c FAILURE
/// once. The behavior tree nodes map it to the BlackThorn status of the same
/// name.
///
/// @code
/// Status MySkill::tick(RobotContext& p_context, Seconds p_dt)
/// {
///     if (reached(p_context))
///         return Status::SUCCESS;
///     return Status::RUNNING;
/// }
/// @endcode
// -------------------------------------------------------------------------
enum class Status
{
    IDLE,    ///< Not started or explicitly idle.
    RUNNING, ///< Still working toward the goal.
    SUCCESS, ///< Goal reached.
    FAILURE, ///< Goal cannot be reached or was aborted.
};

} // namespace robotik
