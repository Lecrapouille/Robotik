// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file Simulation.hpp
/// Runs a scenario file and tells whether it went well.
///
/// A scenario is a robot cell (what to build, see @ref
/// robotik::RoboticCellDescription) plus how its run is judged: a time limit, a stop
/// condition and assertions. The simulation builds the @ref robotik::RoboticCell,
/// steps it, checks the assertions after each step and decides when the run is
/// over. Headless unless a
/// @ref robotik::SceneView is given.
///
/// @par Contract
/// - Promises: the verdicts are frozen when the run ends; @c ended is emitted
///   once; @c assertionChanged when an assertion changes color.
/// - Expects: a scenario read by @ref robotik::loadScenario.
/// - Errors: a scenario whose cell cannot be built throws
///   @c std::runtime_error in the constructor.
#pragma once

#include "RobotIK/Common/Signal.hpp"
#include "RobotIK/Package/Scenario/Assertions.hpp"
#include "RobotIK/Package/Scenario/Scenario.hpp"
#include "RobotIK/Task/RoboticCell.hpp"

#include <cstdint>
#include <string_view>
#include <vector>

namespace robotik
{

// ****************************************************************************
/// One run of a scenario, replayable from a seed.
///
/// A @ref RoboticCell runs robots, but it does not know when a run is over nor if
/// it passed: that belongs to the scenario file. The simulation adds this
/// judgement around a cell. It compiles the assertions and the @c stop_when
/// condition once, measures the metrics once per step, and gives each
/// assertion its verdict. Use it when a run must answer "pass or fail": the
/// hosts, an experiment, a test of a scenario file. A program that only
/// drives robots uses the cell alone.
///
/// The run ends when the controller finishes, the @c stop_when condition
/// holds, the @c timeout elapses or an assertion marked @c stop fails. The
/// physics keeps running afterwards; the verdicts are frozen. @ref ended,
/// @ref assertionChanged and the signals of the cell tell an interface what
/// changed. Pass or fail is never decided in C++: a scenario logic publishes
/// metrics, the assertions of the file judge them.
///
/// From tests/Examples/TestLayerExamples.cpp (PackageScenarioFile):
/// @code
/// auto scenario = robotik::loadScenario("hello.scenario.yaml", assets);
/// HelloLogic logic;
/// compages::world::World world;
/// robotik::Simulation simulation(world, scenario.take(), nullptr, &logic);
/// while (!simulation.finished())
///     simulation.step(robotik::Seconds(0.01));
/// bool const ok = simulation.passed();
/// @endcode
// ****************************************************************************
class Simulation
{
public:

    /// Why a run ended.
    enum class Outcome : std::uint8_t
    {
        Running,             ///< Not ended.
        ControllerSucceeded, ///< The tree, the planner or the scenario logic succeeded.
        ControllerFailed, ///< The tree, the planner or the scenario logic failed.
        StopCondition,    ///< The @c stop_when condition holds.
        Timeout,          ///< The @c timeout elapsed.
        AssertionFailed, ///< An assertion marked @c stop failed.
        /// The host ended the run (@ref conclude).
        Interrupted,
    };

    Simulation(Simulation const&) = delete;
    Simulation& operator=(Simulation const&) = delete;

    // -------------------------------------------------------------------------
    /// Builds the cell of the scenario, then resets with the scenario seed.
    /// @param p_world ECS world receiving the entities.
    /// @param p_scenario Scenario to run.
    /// @param p_view Rendering hooks, or null for a headless run.
    /// @param p_logic Task plugin (skills, detectors, metrics); not owned.
    /// @throws std::runtime_error when the cell cannot be built, or when a
    ///   condition of the assertions or of @c stop_when does not compile
    ///   (an unknown metric, query or entity: see Condition.hpp).
    // -------------------------------------------------------------------------
    Simulation(compages::world::World& p_world,
               Scenario p_scenario,
               SceneView* p_view = nullptr,
               ScenarioLogic* p_logic = nullptr);

    // -------------------------------------------------------------------------
    /// Starts a new episode: same seed, same episode.
    /// @param p_seed Seed of the episode (see @ref RoboticCell::reset).
    // -------------------------------------------------------------------------
    void reset(Seed p_seed);

    /// New episode with the scenario seed.
    void reset()
    {
        reset(Seed{ m_scenario.seed });
    }

    // -------------------------------------------------------------------------
    /// Steps the cell, then checks the assertions and the end of the run.
    /// @param p_dt Duration of the step.
    // -------------------------------------------------------------------------
    void step(Seconds p_dt);

    /// @return The cell being run: robots, skills, beliefs, truth.
    [[nodiscard]] RoboticCell& cell()
    {
        return m_cell;
    }

    /// @copydoc cell()
    [[nodiscard]] RoboticCell const& cell() const
    {
        return m_cell;
    }

    /// @return The scenario being run.
    [[nodiscard]] Scenario const& scenario() const
    {
        return m_scenario;
    }

    /// @return Simulated time since the reset.
    [[nodiscard]] Seconds time() const
    {
        return m_cell.time();
    }

    /// @return Seed of the current episode.
    [[nodiscard]] Seed seed() const
    {
        return m_cell.seed();
    }

    // --- End of run -----------------------------------------------------------

    /// @return Why the run ended, or @c Running.
    [[nodiscard]] Outcome outcome() const
    {
        return m_outcome;
    }

    /// @return True once the run ended (the physics may keep running).
    [[nodiscard]] bool finished() const
    {
        return m_outcome != Outcome::Running;
    }

    // -------------------------------------------------------------------------
    /// Ends the run now (the host's own limit): pending assertions get their
    /// final verdict. No effect on a finished run.
    // -------------------------------------------------------------------------
    void conclude();

    /// @return Assertions with their verdicts (live while running).
    [[nodiscard]] std::vector<Check> const& checks() const
    {
        return m_assertions.checks();
    }

    /// @return True when the run finished and every assertion passed.
    [[nodiscard]] bool passed() const
    {
        return m_assertions.passed();
    }

    // --- Events ---------------------------------------------------------------

    /// Emitted when an assertion turns green or red (see @ref ok), with its
    /// index in @ref checks and its new color. Every assertion is emitted at
    /// the first step of an episode.
    Signal<std::size_t, bool> assertionChanged;

    /// Emitted once when the run ends, with the reason.
    Signal<Outcome> ended;

    // -------------------------------------------------------------------------
    /// Evaluates a condition on the metrics and the truth of the cell (a
    /// one-off question: the assertions are compiled once).
    /// @param p_condition E.g. @c "(inside red_cube box)".
    /// @return What was measured and whether it holds.
    // -------------------------------------------------------------------------
    [[nodiscard]] Evaluation evaluate(std::string_view p_condition) const
    {
        return m_cell.evaluate(p_condition);
    }

private:

    /// Updates the assertions and ends the run when a stop rule holds.
    void settle();
    /// Ends the run: final verdicts, frozen outcome, controller stopped.
    void finish(Outcome p_outcome);
    /// Emits @ref assertionChanged for the assertions whose color changed.
    void announceChecks();

    /// The scenario being run.
    Scenario m_scenario;
    /// The cell built from it.
    RoboticCell m_cell;
    /// Judges the assertions.
    AssertionEngine m_assertions;
    /// The conditions of @ref m_assertions, compiled once.
    std::vector<Condition> m_conditions;
    /// The @c stop_when condition, compiled once; empty when none.
    Condition m_stop_when;
    /// Why the run ended.
    Outcome m_outcome = Outcome::Running;
    /// Last color announced of each assertion: 0 red, 1 green, 2 unknown.
    std::vector<std::uint8_t> m_colors;
};

// ----------------------------------------------------------------------------
/// @param p_outcome Outcome to name.
/// @return For example "timeout" or "controller succeeded".
// ----------------------------------------------------------------------------
[[nodiscard]] char const* toString(Simulation::Outcome p_outcome);

} // namespace robotik
