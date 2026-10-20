// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file Result.hpp
/// How RobotIK says that something could not be done, without exceptions.
///
/// Reading a file, a robot model or a scenario can fail for reasons outside
/// the program: a missing file, a typo in a YAML field. Such a function
/// returns a @c Result: either the value, or a message telling the user what
/// to fix ("worlds/arm.world.yaml: entity 'box': size needs 3 numbers").
/// The caller checks it before use. It is the type Compages uses for the
/// same job, so both libraries report errors the same way.
///
/// A mistake in the program itself (a joint name that does not exist, written
/// in the code) is not a @c Result: it stops the program, because it must be
/// fixed, not handled.
///
/// @code
/// robotik::Result<int> half(int p_value)
/// {
///     if (p_value % 2 != 0)
///         return robotik::failure("odd number: " + std::to_string(p_value));
///     return p_value / 2;
/// }
///
/// robotik::Result<void> run()
/// {
///     ROBOTIK_TRY(half(3));       // gives up and returns the message
///     return robotik::success();
/// }
/// @endcode
///
/// @par Contract
/// - Promises: a @c Result holds a value or a message, never both.
/// - Expects: the caller checks it (@c if (result)) before @c value(); reading
///   the value of a failure is a programming error.
#pragma once

#include "Compages/Core/Result.hpp"

namespace robotik
{

using compages::Failure; ///< An error message on its way to the caller.
using compages::failure; ///< Builds the error returned as any Result.
using compages::Result;  ///< Either a value or the reason it is missing.
using compages::success; ///< A Result<void> that worked.

} // namespace robotik

/// Runs an expression returning a Result and returns its error at once if it
/// failed.
#define ROBOTIK_TRY(expr) COMPAGES_TRY(expr)

/// Moves the value of a Result into @p target, or returns its error at once.
#define ROBOTIK_TRY_ASSIGN(target, expr) COMPAGES_TRY_ASSIGN(target, expr)
