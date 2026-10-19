// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file SkillNodes.hpp
/// Behavior tree actions backed by the @ref SkillScheduler.
#pragma once

#include "BlackThorn/BlackThorn.hpp"

namespace robotik
{

class SkillScheduler;

// ----------------------------------------------------------------------------
/// Registers one BlackThorn action per skill of @p_scheduler, named after its
/// @ref SkillDescription::name.
///
/// The tree decides what should be done, the scheduler what can be done: a
/// tick requests the skill and reports @c RUNNING while it waits or runs,
/// @c SUCCESS once it succeeded, @c FAILURE if it failed, was preempted or
/// cancelled. Resetting the node cancels a run still in progress.
///
/// Call it after every skill is added to the scheduler: a skill added later
/// has no node.
///
/// @code
/// scheduler.add<PickSkill>({ .name = "Pick" });
/// registerSkills(factory, scheduler);
/// // A .bt.yaml can now use the action "Pick".
/// @endcode
///
/// @param p_factory BlackThorn factory that receives the actions.
/// @param p_scheduler Scheduler running the skills. It must outlive the
///   factory and the trees it builds.
// ----------------------------------------------------------------------------
void registerSkills(bt::NodeFactory& p_factory, SkillScheduler& p_scheduler);

} // namespace robotik
