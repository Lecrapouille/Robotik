// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file PlanExecutor.hpp
/// @brief Plans with the PDDL planner, runs the plan as a behavior tree,
/// checks it on the beliefs and plans again when reality disagrees.
///
/// @code
/// state of the beliefs (observers) + facts + objects (kind) + goal
///   -> problem.pddl -> planner -> plan -> behavior tree (one subtree per step)
/// steps that do not depend on each other run together (PlanGraph.hpp)
/// each step: its nodes (skills), then Check: its effects hold in the beliefs
/// a step fails, an effect is false, the goal is not reached
///   -> cancel, wait replan.delay (simulated), plan again (replan.max times)
/// @endcode
///
/// Planning happens inside a simulation step: simulated time does not move
/// during the search, and the search is deterministic (@ref PlanningOptions).
#pragma once

#include "Robotik/Math/Random.hpp"
#include "Robotik/Planning/PlanningDomain.hpp"
#include "Robotik/Planning/PlanningEngine.hpp"
#include "Robotik/Scenario/Scenario.hpp"

#include "BlackThorn/BlackThorn.hpp"

#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace robotik
{

class SkillScheduler;
struct RobotContext;

// ****************************************************************************
/// @brief An object of the problem: a world entity with a @c kind.
// ****************************************************************************
struct PlanningObject
{
    /// Entity id, as in the world file.
    std::string id;
    /// Lower-case name in the PDDL problem.
    std::string name;
    std::string type;
};

/// @brief An atom of the problem and its value in the beliefs.
using ObservedAtom = std::pair<Atom, Truth>;

// ****************************************************************************
/// @brief One call to the planner, kept for the user interface and reports.
// ****************************************************************************
struct PlanningRecord
{
    Seconds time{};
    /// Why this plan: "start", or what went wrong with the previous one.
    std::string cause;
    /// Problem sent to the planner.
    std::string problem;
    PlanningResult result;
    /// Atoms read on the beliefs to build the problem (entity ids).
    std::vector<ObservedAtom> observed;
    /// Behavior tree run for this plan, empty if none.
    std::string tree;
};

// ****************************************************************************
/// @brief Life cycle of the executor within one episode.
// ****************************************************************************
enum class PlanState : std::uint8_t
{
    Planning,  ///< A plan is due at the next tick.
    Waiting,   ///< Waits replan.delay before planning again.
    Executing, ///< Runs the behavior tree of the plan.
    Succeeded, ///< The goal holds in the beliefs.
    Failed,    ///< No plan, or no replan left.
};

[[nodiscard]] char const* toString(PlanState p_state);

// ****************************************************************************
/// @brief Where one step of the current plan is.
// ****************************************************************************
enum class StepStatus : std::uint8_t
{
    Pending, ///< Not reached yet, or running.
    Skipped, ///< Its effects already held in the beliefs: nothing to do.
    Done,    ///< Its skills succeeded and its effects were checked.
    Failed,  ///< A skill failed or an effect was not seen (see failure()).
};

[[nodiscard]] char const* toString(StepStatus p_status);

// ****************************************************************************
/// @brief Drives a @ref PlannerSpec goal through the skills of a robot.
// ****************************************************************************
class PlanExecutor
{
public:

    // -------------------------------------------------------------------------
    /// @brief Loads the domain and checks that the domain, the world, the
    /// scenario and the mission agree (see @ref PlanningDomain).
    /// @param p_name Name of the PDDL problem (the scenario id).
    /// @param p_factory Holds the skill nodes; the executor adds its own.
    /// It and @p_skills must outlive the executor.
    /// @throws std::runtime_error listing every problem found.
    // -------------------------------------------------------------------------
    PlanExecutor(
        PlannerSpec p_spec,
        WorldDescription const& p_world,
        PlanningDomain const& p_domain,
        std::string const& p_name,
        bt::NodeFactory& p_factory,
        SkillScheduler& p_skills,
        std::unique_ptr<PlanningEngine> p_engine = makeKairosPlanner());
    ~PlanExecutor();

    // -------------------------------------------------------------------------
    /// @brief Clock of the trees built from the plans (the simulated time):
    /// their timed nodes then follow it. Empty: the wall clock.
    // -------------------------------------------------------------------------
    void clock(bt::Tree::ClockFunction p_clock)
    {
        m_clock = std::move(p_clock);
    }

    // -------------------------------------------------------------------------
    /// @brief New episode: forgets the plans; the next tick plans.
    // -------------------------------------------------------------------------
    void reset(Seed p_seed);

    // -------------------------------------------------------------------------
    /// @brief Plans when due, then ticks the plan.
    /// @return @c RUNNING, or @c SUCCESS / @c FAILURE once the run is over.
    // -------------------------------------------------------------------------
    bt::Status tick(RobotContext const& p_context);

    // -------------------------------------------------------------------------
    /// @brief Drops the current plan and plans again at the next tick
    /// (user request). Does not count as a replan.
    // -------------------------------------------------------------------------
    void replan(std::string p_cause);

    // -------------------------------------------------------------------------
    /// @brief The problem the planner would get now.
    // -------------------------------------------------------------------------
    [[nodiscard]] std::string problem(RobotContext const& p_context) const;

    // -------------------------------------------------------------------------
    /// @brief Every atom with an observer, read now on the beliefs.
    // -------------------------------------------------------------------------
    [[nodiscard]] std::vector<ObservedAtom>
    observe(RobotContext const& p_context) const;

    // --- Introspection ------------------------------------------------------

    [[nodiscard]] PlannerSpec const& spec() const
    {
        return m_spec;
    }

    [[nodiscard]] DomainSchema const& schema() const
    {
        return m_schema;
    }

    /// @brief Text of the domain file.
    [[nodiscard]] std::string const& domain() const
    {
        return m_domain_text;
    }

    [[nodiscard]] std::vector<PlanningObject> const& objects() const
    {
        return m_objects;
    }

    [[nodiscard]] std::vector<PlanningRecord> const& history() const
    {
        return m_history;
    }

    /// @brief Steps of the current plan (entity ids as arguments).
    [[nodiscard]] std::vector<GroundAction> const& plan() const
    {
        return m_plan;
    }

    /// @brief Count of steps of the current plan done (or skipped) and
    /// checked; with parallel steps, not always the first ones.
    [[nodiscard]] std::size_t done() const
    {
        return m_done;
    }

    // -------------------------------------------------------------------------
    /// @brief Direct predecessors of each step of the current plan: a step
    /// starts once they are done (see PlanGraph.hpp). A chain when the plan
    /// runs step by step.
    // -------------------------------------------------------------------------
    [[nodiscard]] std::vector<std::vector<std::size_t>> const& after() const
    {
        return m_after;
    }

    /// @brief Status of each step of the current plan.
    [[nodiscard]] std::vector<StepStatus> const& steps() const
    {
        return m_steps;
    }

    [[nodiscard]] PlanState state() const
    {
        return m_state;
    }

    /// @brief Last thing that went wrong, empty if none.
    [[nodiscard]] std::string const& failure() const
    {
        return m_failure;
    }

    [[nodiscard]] std::uint32_t replans() const
    {
        return m_replans;
    }

    /// @brief Tree of the current plan, or null.
    [[nodiscard]] bt::Tree const* tree() const
    {
        return m_tree.get();
    }

    /// @brief Behavior tree nodes of one plan step.
    [[nodiscard]] std::vector<std::string>
    nodes(GroundAction const& p_step) const;

private:

    void load(WorldDescription const& p_world, PlanningDomain const& p_domain);
    void plan(RobotContext const& p_context, std::string p_cause);
    void compile();
    void fail(std::string p_why);
    void retry(RobotContext const& p_context, std::string p_cause);
    [[nodiscard]] bt::Status check(std::size_t p_step,
                                   RobotContext const& p_context);
    [[nodiscard]] bt::Status skip(std::size_t p_step,
                                  RobotContext const& p_context);
    /// @brief SUCCESS once the predecessors of @p_step are done, else RUNNING.
    [[nodiscard]] bt::Status ready(std::size_t p_step) const;
    /// @brief Marks @p_step finished (done or skipped) and counts it.
    void finish(std::size_t p_step, StepStatus p_status);
    /// @brief Step a failed tree failed at: one with a failed skill, else
    /// the first one running.
    [[nodiscard]] std::size_t failingStep() const;
    /// @brief Partial order of @ref m_plan (a chain unless spec.parallel).
    [[nodiscard]] std::vector<std::vector<std::size_t>> order() const;
    /// @brief Plain effects of a step, as ground literals (PDDL names).
    [[nodiscard]] std::vector<Literal> effects(std::size_t p_step,
                                               bool& p_partial) const;
    [[nodiscard]] Truth evaluate(Atom const& p_atom,
                                 RobotContext const& p_context) const;
    [[nodiscard]] std::string goalReached(RobotContext const& p_context) const;
    [[nodiscard]] std::string stepFailure(RobotContext const& p_context) const;
    [[nodiscard]] std::vector<std::string>
    ids(std::vector<std::string> const& p_names) const;
    [[nodiscard]] std::string
    problemOf(std::vector<ObservedAtom> const& p_observed) const;

private:

    PlannerSpec m_spec;
    std::string m_name;
    PlanningDomain const& m_domain;
    bt::NodeFactory& m_factory;
    SkillScheduler& m_skills;
    std::unique_ptr<PlanningEngine> m_engine;

    std::string m_domain_text;
    DomainSchema m_schema;
    std::vector<PlanningObject> m_objects;

    std::uint64_t m_seed = 0u;
    PlanState m_state = PlanState::Planning;
    Seconds m_resume{};
    std::uint32_t m_replans = 0u;
    std::string m_failure;
    std::vector<PlanningRecord> m_history;
    std::vector<GroundAction> m_plan;
    std::size_t m_done = 0u;
    std::vector<StepStatus> m_steps;
    std::vector<std::vector<std::size_t>> m_after;
    /// Set by a failed check: why the tree failed.
    std::string m_check_failure;
    /// Context of the tick in progress, for the check nodes.
    RobotContext const* m_context = nullptr;
    bt::Blackboard::Ptr m_blackboard;
    bt::Tree::Ptr m_tree;
    bt::Tree::ClockFunction m_clock;
};

} // namespace robotik
