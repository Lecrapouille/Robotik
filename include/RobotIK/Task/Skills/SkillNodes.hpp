// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file SkillNodes.hpp
/// Behavior tree nodes that run a skill through the scheduler.
///
/// This is the only RobotIK header that includes BlackThorn. Include it to
/// write your own tree nodes (@ref robotik::RoboticCell::factory) or to walk a tree
/// (@ref robotik::RoboticCell::tree); the rest of the API names the tree types only
/// (TreeTypes.hpp) and answers with @ref robotik::Status.
///
/// The tree decides what to do; the node only asks the scheduler, so resources
/// and priorities stay checked (see doc/reference/glossary.md: "Behavior
/// tree").
///
/// @par Contract
/// - Promises: a node asks the scheduler for its skill and returns its state; a
///   halted node cancels the skill.
/// - Expects: the skill is registered in the scheduler.
#pragma once

#include "RobotIK/Task/Status.hpp"

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

// ----------------------------------------------------------------------------
/// Converts a BlackThorn status into the RobotIK one.
/// @param p_status Status of a tree or a node; @c INVALID (not ticked yet)
///   becomes @c IDLE.
/// @return The status of the same name.
// ----------------------------------------------------------------------------
[[nodiscard]] inline Status toStatus(bt::Status p_status)
{
    switch (p_status)
    {
        case bt::Status::RUNNING:
            return Status::RUNNING;
        case bt::Status::SUCCESS:
            return Status::SUCCESS;
        case bt::Status::FAILURE:
            return Status::FAILURE;
        case bt::Status::INVALID:
            break;
    }
    return Status::IDLE;
}

} // namespace robotik
