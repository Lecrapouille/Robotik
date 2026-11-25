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

// ****************************************************************************
/// Tree node of a skill. It asks for its own run once, then follows that run
/// only: another caller of the same skill has its own ticket. When the run
/// ends, it writes "<skill>.state" and "<skill>.reason" on the blackboard, so
/// the tree can tell a failure from a cancel or a lost resource.
// ****************************************************************************
class SkillLeaf final: public bt::CallbackLeaf
{
public:

    SkillLeaf(SkillScheduler& p_scheduler, SkillId p_id)
        : CallbackLeaf([]() noexcept { return bt::Status::FAILURE; }),
          m_scheduler(p_scheduler),
          m_id(p_id)
    {
    }

    void fillConfig(bt::NodeConfig& p_config) const override
    {
        SkillLeaf* self = const_cast<SkillLeaf*>(this);
        p_config.callback = [self]() { return self->onTick(); };
        p_config.on_reset = [self]() { self->onHalt(); };
    }

private:

    /// Asks for the run, then answers its state.
    bt::Status onTick()
    {
        if (m_run == NO_RUN)
        {
            m_run = m_scheduler.request(m_id);
            return bt::Status::RUNNING;
        }
        SkillState const state = m_scheduler.state(m_run);
        switch (state)
        {
            case SkillState::Idle:
            case SkillState::Waiting:
            case SkillState::Running:
            case SkillState::Stopping:
                return bt::Status::RUNNING;
            case SkillState::Succeeded:
            case SkillState::Failed:
            case SkillState::Cancelled:
                break;
        }
        tell(state, m_scheduler.reason(m_run));
        m_run = NO_RUN;
        return state == SkillState::Succeeded ? bt::Status::SUCCESS
                                              : bt::Status::FAILURE;
    }

    /// The tree moves on: cancels this node's run only.
    void onHalt()
    {
        if (m_run != NO_RUN)
        {
            m_scheduler.cancel(m_run);
            m_run = NO_RUN;
        }
    }

    /// Writes how the run ended on the blackboard of the tree.
    void tell(SkillState p_state, SkillReason p_reason) const
    {
        if (bt::Blackboard::Ptr const board = blackboard())
        {
            std::string const& skill = m_scheduler.name(m_id);
            board->set(skill + ".state",
                       std::string(robotik::toString(p_state)));
            board->set(skill + ".reason",
                       std::string(robotik::toString(p_reason)));
        }
    }

    SkillScheduler& m_scheduler;
    SkillId m_id;
    /// Ticket of the run this node asked for, or NO_RUN.
    RunId m_run{};
};

void registerSkills(bt::NodeFactory& p_factory, SkillScheduler& p_scheduler)
{
    for (SkillId id = 0; id < p_scheduler.size(); ++id)
    {
        // One ticket per tree node: the same skill may appear twice.
        p_factory.registerNode(p_scheduler.name(id),
                               [&p_scheduler, id]()
                               {
                                   return std::make_unique<SkillLeaf>(
                                       p_scheduler, id);
                               });
    }
}

} // namespace robotik
