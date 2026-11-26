// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file StoryReader.hpp
/// Reads the @c story: section of a scenario file (doc/reference/story-file.md)
/// into a @ref robotik::StorySpec. Used by Scenario.cpp only.
///
/// @par Contract
/// - Promises: an unknown key, action or gait is a message that names the
///   event and the action; nothing is thrown.
/// - Expects: the node is the @c story: map, or null (no story).
/// - Errors: a @c Result. Names of actors, places and events are checked
///   later, with the world, by @c validate.
#pragma once

#include "RobotIK/Common/Config.hpp"
#include "RobotIK/Common/Result.hpp"
#include "RobotIK/Task/Story/StoryDescription.hpp"

namespace robotik
{

// ----------------------------------------------------------------------------
/// @param p_node The @c story: map of a scenario, or a null node.
/// @return The story; empty for a null node.
// ----------------------------------------------------------------------------
[[nodiscard]] Result<StorySpec> readStory(ConfigNode const& p_node);

} // namespace robotik
