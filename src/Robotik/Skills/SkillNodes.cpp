// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "Robotik/Skills/SkillNodes.hpp"

#include "Robotik/Runtime/Scheduler.hpp"

#include <memory>

namespace robotik
{

static bt::Status tickSkill(SkillScheduler& p_scheduler, SkillId p_id, bool& p_active)
{
    if (!p_active)
    {
        p_scheduler.request(p_id);
        p_active = true;
        return bt::Status::RUNNING;
    }
    switch (p_scheduler.state(p_id))
    {
        case SkillState::Idle:
        case SkillState::Waiting:
        case SkillState::Running:
            return bt::Status::RUNNING;
        case SkillState::Succeeded:
            p_active = false;
            return bt::Status::SUCCESS;
        case SkillState::Failed:
        case SkillState::Cancelled:
            p_active = false;
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
                // One flag per tree node: the same skill may appear twice.
                auto active = std::make_shared<bool>(false);
                return std::make_unique<bt::CallbackLeaf>(
                    [&p_scheduler, id, active]()
                    { return tickSkill(p_scheduler, id, *active); },
                    [&p_scheduler, id, active]()
                    {
                        if (*active)
                        {
                            p_scheduler.cancel(id);
                            *active = false;
                        }
                    });
            });
    }
}

} // namespace robotik
