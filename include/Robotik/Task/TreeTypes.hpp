// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file TreeTypes.hpp
/// Names of the behavior tree types, without the behavior tree library.
///
/// A @ref robotik::Cell and a @ref robotik::PlanExecutor run behavior
/// trees (BlackThorn), but most users only read a status or a plan. These
/// declarations let the public headers name the tree types without
/// including BlackThorn, so an application that does not write tree nodes
/// never sees it.
///
/// When: include @c Robotik/Task/Skills/SkillNodes.hpp instead to write your
/// own nodes or to walk a tree; it brings BlackThorn and @ref toStatus.
///
/// @par Contract
/// - Promises: the names match BlackThorn (@c bt::Status is an @c int enum,
///   @ref robotik::TreeClock is @c bt::Tree::ClockFunction).
#pragma once

#include <chrono>
#include <functional>

namespace bt
{
class Blackboard;
class Node;
class NodeFactory;
class Tree;
enum class Status;
} // namespace bt

namespace robotik
{

/// Clock of the timed tree nodes (@c Timeout, @c Delay...): the simulated
/// time in a @ref Cell. Same type as @c bt::Tree::ClockFunction.
using TreeClock = std::function<std::chrono::steady_clock::time_point()>;

} // namespace robotik
