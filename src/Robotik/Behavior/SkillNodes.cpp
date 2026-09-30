// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "Robotik/Behavior/SkillNodes.hpp"

#include "Robotik/Runtime/RobotContext.hpp"
#include "Robotik/Skills/Skill.hpp"

namespace robotik
{

namespace
{

bt::Status toTree(Status p_status)
{
    switch (p_status)
    {
        case Status::Success:
            return bt::Status::SUCCESS;
        case Status::failure:
            return bt::Status::FAILURE;
        case Status::Running:
        case Status::Idle:
            return bt::Status::RUNNING;
    }
    return bt::Status::FAILURE;
}

} // namespace

void registerSkill(bt::NodeFactory& p_factory,
                   std::string const& p_name,
                   std::shared_ptr<Skill> p_skill,
                   RobotContext& p_context,
                   SkillTrace& p_trace)
{
    // Index of the trace entry while the skill runs, -1 when it is idle.
    auto running = std::make_shared<long>(-1);
    p_factory.registerAction(
        p_name,
        [p_name, p_skill, running, &p_context, &p_trace]()
        {
            if (*running < 0)
            {
                p_skill->reset();
                *running = static_cast<long>(p_trace.entries.size());
                p_trace.entries.push_back(
                    { p_name, Status::Running, p_context.time, p_context.time });
            }
            Status const status = p_skill->tick(p_context, p_context.dt);
            SkillTrace::Entry& entry =
                p_trace.entries[static_cast<std::size_t>(*running)];
            entry.status = status;
            entry.end = p_context.time;
            if (status != Status::Running)
            {
                *running = -1;
            }
            return toTree(status);
        });
}

} // namespace robotik
