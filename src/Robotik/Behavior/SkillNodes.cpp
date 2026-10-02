#include "Robotik/Behavior/SkillNodes.hpp"

#include "Robotik/Runtime/RobotContext.hpp"
#include "Robotik/Skills/Skill.hpp"

namespace robotik
{

// -------------------------------------------------------------------------
// @brief Converts a @ref robotik::Status to a @ref bt::Status.
// -------------------------------------------------------------------------
bt::Status toTree(Status p_status)
{
    switch (p_status)
    {
        case Status::SUCCESS:
            return bt::Status::SUCCESS;
        case Status::FAILURE:
            return bt::Status::FAILURE;
        case Status::RUNNING:
        case Status::IDLE:
            return bt::Status::RUNNING;
    }
    return bt::Status::FAILURE;
}

// -------------------------------------------------------------------------
//! @brief One BlackThorn action tick: trace, skill @ref Skill::tick, map
//! status.
// -------------------------------------------------------------------------
static bt::Status tickSkillAction(std::string const& p_name,
                                  std::shared_ptr<Skill> const& p_skill,
                                  std::shared_ptr<long> const& p_running,
                                  RobotContext& p_context,
                                  SkillTrace& p_trace)
{
    // New BT action run: idle since last SUCCESS/FAILURE (p_running == -1).
    if (*p_running < 0)
    {
        p_skill->reset();
        *p_running = static_cast<long>(p_trace.entries.size());
        p_trace.entries.emplace_back(
            p_name, Status::RUNNING, p_context.time, p_context.time);
    }

    // Advance skill one simulation step; context carries world, time, and dt.
    Status const status = p_skill->tick(p_context, p_context.dt);

    // Refresh the open trace row for this run (same index until the run ends).
    SkillTrace::Entry& entry =
        p_trace.entries[static_cast<std::size_t>(*p_running)];
    entry.status = status;
    entry.end = p_context.time;

    // Run finished: next visit will reset the skill and append a new entry.
    if (status != Status::RUNNING)
    {
        *p_running = -1;
    }

    return toTree(status);
}

void registerSkill(bt::NodeFactory& p_factory,
                   std::string const& p_name,
                   std::shared_ptr<Skill> p_skill,
                   RobotContext& p_context,
                   SkillTrace& p_trace)
{
    // Per-action state: index of the open SkillTrace row, or -1 between runs.
    auto running = std::make_shared<long>(-1);

    // BlackThorn stores this lambda inside bt::CallbackLeaf (see Factory.hpp:
    // registerAction -> make_unique<CallbackLeaf>(func)). Each time the
    // interpreter ticks the YAML Action named p_name, it invokes that function;
    // the lambda below is therefore the BT action entry point.
    p_factory.registerAction(
        p_name,
        [p_name, p_skill, running, &p_context, &p_trace]()
        {
            return tickSkillAction(
                p_name, p_skill, running, p_context, p_trace);
        });
}

} // namespace robotik
