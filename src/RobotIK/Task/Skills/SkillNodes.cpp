// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "RobotIK/Task/Skills/SkillNodes.hpp"

#include "RobotIK/Task/Scheduler.hpp"

#include <memory>

namespace robotik
{

/// Ticks one tree node: asks for its own run once, then follows that run
/// only. Another caller of the same skill has its own ticket.
static bt::Status
tickSkill(SkillScheduler& p_scheduler, SkillId p_id, RunId& p_run)
{
    if (p_run == NO_RUN)
    {
        p_run = p_scheduler.request(p_id);
        return bt::Status::RUNNING;
    }
    switch (p_scheduler.state(p_run))
    {
        case SkillState::Idle:
        case SkillState::Waiting:
        case SkillState::Running:
        case SkillState::Stopping:
            return bt::Status::RUNNING;
        case SkillState::Succeeded:
            p_run = NO_RUN;
            return bt::Status::SUCCESS;
        case SkillState::Failed:
        case SkillState::Cancelled:
            p_run = NO_RUN;
            return bt::Status::FAILURE;
    }
    return bt::Status::FAILURE;
}

void registerSkills(bt::NodeFactory& p_factory, SkillScheduler& p_scheduler)
{
    for (SkillId id = 0; id < p_scheduler.size(); ++id)
    {
        p_factory.registerNode(
            p_scheduler.name(id),
            [&p_scheduler, id]()
            {
                // One ticket per tree node: the same skill may appear twice.
                auto run = std::make_shared<RunId>();
                return std::make_unique<bt::CallbackLeaf>(
                    [&p_scheduler, id, run]()
                    { return tickSkill(p_scheduler, id, *run); },
                    [&p_scheduler, run]()
                    {
                        if (*run != NO_RUN)
                        {
                            p_scheduler.cancel(*run);
                            *run = NO_RUN;
                        }
                    });
            });
    }
}

} // namespace robotik
