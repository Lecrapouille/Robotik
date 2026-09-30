/**
 * @file SkillNodes.hpp
 * @brief Registers @ref Skill instances as BlackThorn action nodes and traces runs.
 */

#pragma once

#include "Robotik/Runtime/Status.hpp"

#include "BlackThorn/BlackThorn.hpp"

#include <memory>
#include <string>
#include <vector>

namespace robotik
{

class Skill;
struct RobotContext;

/**
 * @brief Log of skill executions, oldest entry first.
 *
 * Filled by @ref registerSkill for operator UI and post-mortem debugging.
 */
struct SkillTrace
{
    /** @brief One invocation of a registered BT action. */
    struct Entry
    {
        /** @brief BlackThorn action name passed to @ref registerSkill. */
        std::string name;

        /** @brief Last status reported by the skill for this run. */
        Status status = Status::Running;

        /** @brief @ref RobotContext::time when the run started. */
        double start = 0.0;

        /** @brief @ref RobotContext::time after the last tick of this run. */
        double end = 0.0;
    };

    /** @brief Chronological list of skill runs. */
    std::vector<Entry> entries;
};

/**
 * @brief Registers @p_p_name as a BT action that ticks @p_skill each visit.
 *
 * The skill is @ref Skill::reset when a new run starts. Status is mapped to
 * @c bt::Status and appended to @p_trace.
 *
 * @param p_factory Node factory to extend.
 * @param p_name Action type name used in YAML (e.g. @c "Home").
 * @param p_skill Shared skill instance (may be reused across ticks).
 * @param p_context Context whose @c time and @c dt are read each tick.
 * @param p_trace Trace buffer updated on start and each tick.
 */
void registerSkill(bt::NodeFactory& p_factory,
                   std::string const& p_name,
                   std::shared_ptr<Skill> p_skill,
                   RobotContext& p_context,
                   SkillTrace& p_trace);

} // namespace robotik
