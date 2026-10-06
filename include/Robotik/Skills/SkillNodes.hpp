// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

//! @file SkillNodes.hpp
//! @brief Behavior tree actions backed by the @ref SkillScheduler.
#pragma once

#include "BlackThorn/BlackThorn.hpp"

namespace robotik
{

class SkillScheduler;

// ----------------------------------------------------------------------------
//! @brief Registers one BlackThorn action per skill of @p_scheduler, named
//! after its @ref SkillDescription::name.
//!
//! The tree decides what should be done, the scheduler what can be done: a
//! tick requests the skill and reports @c RUNNING while it waits or runs,
//! @c SUCCESS once it succeeded, @c FAILURE if it failed, was preempted or
//! cancelled. Resetting the node cancels a run still in progress.
//!
//! @p_scheduler must outlive the factory and the trees it builds.
// ----------------------------------------------------------------------------
void registerSkills(bt::NodeFactory& p_factory, SkillScheduler& p_scheduler);

} // namespace robotik
