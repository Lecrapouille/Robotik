// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file PlanExecutor.hpp
/// Plans with the PDDL planner, runs the plan as a behavior tree, checks it
/// on the beliefs and plans again when reality disagrees.
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

#include "Robotik/Common/Random.hpp"
#include "Robotik/Task/Planning/PlanGraph.hpp"
#include "Robotik/Task/Planning/PlanningDomain.hpp"
#include "Robotik/Task/Planning/PlanningEngine.hpp"
#include "Robotik/Task/CellDescription.hpp"

#include "BlackThorn/BlackThorn.hpp"

#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace robotik
{

class SkillScheduler;
struct RobotContext;

// ****************************************************************************
/// An object of the problem: a world entity with a @c kind.
///
/// @code
/// for (robotik::PlanningObject const& o : executor.objects())
///     std::cout << o.name << " - " << o.type << "   ; entity " << o.id <<
///     '\n';
/// @endcode
// ****************************************************************************
struct PlanningObject
{
    /// Entity id, as in the world file.
    std::string id;
    /// Lower-case name in the PDDL problem.
    std::string name;
    /// PDDL type (the @c kind of the entity).
    std::string type;
};

/// An atom of the problem and its value in the beliefs.
using ObservedAtom = std::pair<Atom, Truth>;

// ****************************************************************************
/// One call to the planner, kept for the user interface and reports.
///
/// @code
/// robotik::PlanningRecord const& last = executor.history().back();
/// std::cout << last.cause << " -> " << robotik::toString(last.result.status)
/// << '\n'
///           << last.problem;
/// @endcode
// ****************************************************************************
struct PlanningRecord
{
    /// Simulated time of the call.
    Seconds time{};
    /// Why this plan: "start", or what went wrong with the previous one.
    std::string cause;
    /// Problem sent to the planner.
    std::string problem;
    /// Answer of the planner.
    PlanningResult result;
    /// Atoms read on the beliefs to build the problem (entity ids).
    std::vector<ObservedAtom> observed;
    /// Behavior tree run for this plan, empty if none.
    std::string tree;
};

// ****************************************************************************
/// Life cycle of the executor within one episode.
// ****************************************************************************
enum class PlanState : std::uint8_t
{
    Planning,  ///< A plan is due at the next tick.
    Waiting,   ///< Waits replan.delay before planning again.
    Executing, ///< Runs the behavior tree of the plan.
    Succeeded, ///< The goal holds in the beliefs.
    Failed,    ///< No plan, or no replan left.
};

// ----------------------------------------------------------------------------
/// @param p_state State to name.
/// @return Its name, for the user interface and reports.
// ----------------------------------------------------------------------------
[[nodiscard]] char const* toString(PlanState p_state);

// ****************************************************************************
/// Where one step of the current plan is.
// ****************************************************************************
enum class StepStatus : std::uint8_t
{
    Pending, ///< Not reached yet, or running.
    Skipped, ///< Its effects already held in the beliefs: nothing to do.
    Done,    ///< Its skills succeeded and its effects were checked.
    Failed,  ///< A skill failed or an effect was not seen (see failure()).
};

// ----------------------------------------------------------------------------
/// @param p_status Status to name.
/// @return Its name, for the user interface and reports.
// ----------------------------------------------------------------------------
[[nodiscard]] char const* toString(StepStatus p_status);

// ****************************************************************************
/// Drives a @ref PlannerSpec goal through the skills of a robot.
///
/// The @ref Simulation builds it when the scenario has a planner, and ticks
/// it every step instead of a behavior tree.
///
/// @code
/// robotik::PlanExecutor executor(*scenario.controller.planner, scenario.world,
///                                mission_domain, scenario.id, factory,
///                                skills);
/// executor.reset(robotik::Seed(42));
/// while (executor.tick(context) == bt::Status::RUNNING)
///     step();
/// if (executor.state() == robotik::PlanState::Failed)
///     std::cerr << executor.failure() << '\n';
/// @endcode
// ****************************************************************************
class PlanExecutor
{
public:

    // -------------------------------------------------------------------------
    /// Loads the domain and checks that the domain, the world, the scenario
    /// and the mission agree (see @ref PlanningDomain).
    /// @param p_spec Goal and settings of the planner.
    /// @param p_world World: the entities with a @c kind are the objects.
    /// @param p_domain Observers and bindings of the mission.
    /// @param p_name Name of the PDDL problem (the scenario id).
    /// @param p_factory Holds the skill nodes; the executor adds its own.
    ///   It and @p_skills must outlive the executor.
    /// @param p_skills Scheduler running the skills of the plan.
    /// @param p_engine Planner; Kairos by default.
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
    /// Sets the age after which a belief is unknown to the planner: an
    /// observer naming an object seen longer ago answers @c Unknown.
    /// @param p_age Largest age; infinite by default.
    // -------------------------------------------------------------------------
    void maxAge(Seconds p_age)
    {
        m_max_age = p_age;
    }

    // -------------------------------------------------------------------------
    /// Sets the clock of the trees built from the plans (the simulated
    /// time): their timed nodes then follow it.
    /// @param p_clock Clock function; empty: the wall clock.
    // -------------------------------------------------------------------------
    void clock(bt::Tree::ClockFunction p_clock)
    {
        m_clock = std::move(p_clock);
    }

    // -------------------------------------------------------------------------
    /// New episode: forgets the plans; the next tick plans.
    /// @param p_seed Seed of the episode, given to the planner.
    // -------------------------------------------------------------------------
    void reset(Seed p_seed);

    // -------------------------------------------------------------------------
    /// Plans when due, then ticks the plan.
    /// @param p_context Robot API and beliefs of this tick.
    /// @return @c RUNNING, or @c SUCCESS / @c FAILURE once the run is over.
    // -------------------------------------------------------------------------
    bt::Status tick(RobotContext const& p_context);

    // -------------------------------------------------------------------------
    /// Drops the current plan and plans again at the next tick (user
    /// request). Does not count as a replan.
    /// @param p_cause Why, kept in @ref history.
    // -------------------------------------------------------------------------
    void replan(std::string p_cause);

    // -------------------------------------------------------------------------
    /// @param p_context Robot API and beliefs.
    /// @return The PDDL problem the planner would get now.
    // -------------------------------------------------------------------------
    [[nodiscard]] std::string problem(RobotContext const& p_context) const;

    // -------------------------------------------------------------------------
    /// @param p_context Robot API and beliefs.
    /// @return Every atom with an observer, read now on the beliefs.
    // -------------------------------------------------------------------------
    [[nodiscard]] std::vector<ObservedAtom>
    observe(RobotContext const& p_context) const;

    // --- Introspection ------------------------------------------------------

    /// @return Goal and settings of the planner.
    [[nodiscard]] PlannerSpec const& spec() const
    {
        return m_spec;
    }

    /// @return What RobotIK read of the domain.
    [[nodiscard]] DomainSchema const& schema() const
    {
        return m_schema;
    }

    /// @return Text of the domain file.
    [[nodiscard]] std::string const& domain() const
    {
        return m_domain_text;
    }

    /// @return Objects of the problem.
    [[nodiscard]] std::vector<PlanningObject> const& objects() const
    {
        return m_objects;
    }

    /// @return Every call to the planner since the reset.
    [[nodiscard]] std::vector<PlanningRecord> const& history() const
    {
        return m_history;
    }

    /// @return Steps of the current plan (entity ids as arguments).
    [[nodiscard]] std::vector<GroundAction> const& plan() const
    {
        return m_plan;
    }

    /// @return Count of steps of the current plan done (or skipped) and
    ///   checked; with parallel steps, not always the first ones.
    [[nodiscard]] std::size_t done() const
    {
        return m_done;
    }

    // -------------------------------------------------------------------------
    /// @return Steps each step of the current plan waits for before it
    ///   starts (their end, or their start in a temporal plan). A chain when
    ///   the plan runs step by step.
    // -------------------------------------------------------------------------
    [[nodiscard]] std::vector<std::vector<std::size_t>> const& after() const
    {
        return m_after;
    }

    /// @return True when step @p_step has started and is not over: several
    ///   at once when the plan runs steps together.
    [[nodiscard]] bool running(std::size_t p_step) const;

    /// @return Events each step waits for before it starts and before it
    ///   ends (see PlanGraph.hpp).
    [[nodiscard]] std::vector<StepOrder> const& order() const
    {
        return m_order;
    }

    /// @return Status of each step of the current plan.
    [[nodiscard]] std::vector<StepStatus> const& steps() const
    {
        return m_steps;
    }

    /// @return Where the executor is in the episode.
    [[nodiscard]] PlanState state() const
    {
        return m_state;
    }

    /// @return Last thing that went wrong, empty if none.
    [[nodiscard]] std::string const& failure() const
    {
        return m_failure;
    }

    /// @return Number of replans since the reset.
    [[nodiscard]] std::uint32_t replans() const
    {
        return m_replans;
    }

    /// @return Tree of the current plan, or null.
    [[nodiscard]] bt::Tree const* tree() const
    {
        return m_tree.get();
    }

    /// @param p_step A step of a plan.
    /// @return Behavior tree nodes of the step, arguments replaced.
    [[nodiscard]] std::vector<std::string>
    nodes(GroundAction const& p_step) const;

private:

    /// Reads the domain and checks it against the world and the mission.
    void load(WorldDescription const& p_world, PlanningDomain const& p_domain);
    /// Builds the problem, calls the planner and compiles the plan.
    void plan(RobotContext const& p_context, std::string p_cause);
    /// Orders the steps and builds the behavior tree of the plan.
    void compile();
    /// Ends the episode in @ref PlanState::Failed.
    void fail(std::string p_why);
    /// Waits replan.delay then plans again, or fails when no replan is left.
    void retry(RobotContext const& p_context, std::string p_cause);
    /// Tree node: SUCCESS when the effects of @p_step hold (or verify is off).
    [[nodiscard]] bt::Status check(std::size_t p_step,
                                   RobotContext const& p_context);
    /// Tree node: SUCCESS when every effect of @p_step already holds.
    [[nodiscard]] bt::Status skip(std::size_t p_step,
                                  RobotContext const& p_context);
    /// SUCCESS once @p_step may start (marks it started), else RUNNING.
    [[nodiscard]] bt::Status ready(std::size_t p_step);
    /// SUCCESS once @p_step may end, else RUNNING.
    [[nodiscard]] bt::Status mayEnd(std::size_t p_step) const;
    /// The event happened: a start once the step started, an end once it is
    /// done or skipped.
    [[nodiscard]] bool reached(PlanEvent const& p_event) const;
    /// Marks @p_step finished (done or skipped) and counts it.
    void finish(std::size_t p_step, StepStatus p_status);
    /// Step a failed tree failed at: one with a failed skill, else the first
    /// one running.
    [[nodiscard]] std::size_t failingStep() const;
    /// Partial order of @ref m_plan (a chain unless spec.parallel).
    [[nodiscard]] std::vector<StepOrder> partialOrder() const;
    /// Plain effects of a step, as ground literals (PDDL names);
    /// @p_partial tells that some effects cannot be checked.
    [[nodiscard]] std::vector<Literal> effects(std::size_t p_step,
                                               bool& p_partial) const;
    /// True when the atom is known and every belief it names was observed
    /// at or after @p_since (the @c observed check).
    [[nodiscard]] bool seenSince(Atom const& p_atom,
                                 RobotContext const& p_context,
                                 Seconds p_since) const;
    /// Value of a ground atom: a fact, or its observer on the beliefs.
    [[nodiscard]] Truth evaluate(Atom const& p_atom,
                                 RobotContext const& p_context) const;
    /// Empty when the goal holds; else which goal literal is false.
    [[nodiscard]] std::string goalReached(RobotContext const& p_context) const;
    /// Explains why the tree failed: the step and its failed skill.
    [[nodiscard]] std::string stepFailure(RobotContext const& p_context) const;
    /// Entity ids of PDDL object names (unknown names are kept).
    [[nodiscard]] std::vector<std::string>
    ids(std::vector<std::string> const& p_names) const;
    /// PDDL problem from observed atoms, facts, objects and goal.
    [[nodiscard]] std::string
    problemOf(std::vector<ObservedAtom> const& p_observed) const;

private:

    /// Goal and settings of the planner.
    PlannerSpec m_spec;
    /// Name of the PDDL problem.
    std::string m_name;
    /// Observers and bindings of the mission.
    PlanningDomain const& m_domain;
    /// Factory of the tree nodes.
    bt::NodeFactory& m_factory;
    /// Scheduler running the skills.
    SkillScheduler& m_skills;
    /// The planner.
    std::unique_ptr<PlanningEngine> m_engine;

    /// Text of the domain file.
    std::string m_domain_text;
    /// What RobotIK read of the domain.
    DomainSchema m_schema;
    /// Objects of the problem.
    std::vector<PlanningObject> m_objects;

    /// Seed of the episode.
    std::uint64_t m_seed = 0U;
    /// Where the executor is in the episode.
    PlanState m_state = PlanState::Planning;
    /// Simulated time to plan again, in @ref PlanState::Waiting.
    Seconds m_resume{};
    /// Replans since the reset.
    std::uint32_t m_replans = 0U;
    /// Last thing that went wrong.
    std::string m_failure;
    /// Every call to the planner.
    std::vector<PlanningRecord> m_history;
    /// Steps of the current plan.
    std::vector<GroundAction> m_plan;
    /// Steps done or skipped.
    std::size_t m_done = 0U;
    /// Status of each step.
    std::vector<StepStatus> m_steps;
    /// Steps each step waits for before it starts.
    std::vector<std::vector<std::size_t>> m_after;
    /// Partial order of the plan.
    std::vector<StepOrder> m_order;
    /// The start of each step happened.
    std::vector<bool> m_started;
    /// Simulated time each step started.
    std::vector<Seconds> m_start_times;
    /// A belief older than this is unknown (see @ref maxAge).
    Seconds m_max_age{ std::numeric_limits<double>::infinity() };
    /// Set by a failed check: why the tree failed.
    std::string m_check_failure;
    /// Context of the tick in progress, for the check nodes.
    RobotContext const* m_context = nullptr;
    /// Blackboard of the tree (step parameters).
    bt::Blackboard::Ptr m_blackboard;
    /// Tree of the current plan.
    bt::Tree::Ptr m_tree;
    /// Clock given to the trees.
    bt::Tree::ClockFunction m_clock;
};

} // namespace robotik
