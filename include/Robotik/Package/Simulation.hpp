// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file Simulation.hpp
/// Runs a scenario file and tells whether it went well.
///
/// A scenario is a robot cell (what to build, see @ref CellDescription) plus
/// how its run is judged: a time limit, a stop condition and assertions. The
/// simulation builds the @ref Cell, steps it, checks the assertions after
/// each step and decides when the run is over. Headless unless a
/// @ref SceneView is given.
#pragma once

#include "Robotik/Package/Scenario/Assertions.hpp"
#include "Robotik/Package/Scenario/Scenario.hpp"
#include "Robotik/Task/Cell.hpp"

#include <cstdint>
#include <string_view>
#include <vector>

namespace robotik
{

// ****************************************************************************
/// One run of a scenario, replayable from a seed.
///
/// The run ends when the controller finishes, the @c stop_when condition
/// holds, the @c timeout elapses or an assertion marked @c stop fails. The
/// physics keeps running afterwards; the verdicts are frozen.
///
/// @code
/// compages::world::World world;
/// robotik::Simulation sim(world, robotik::Scenario::load(path, assets));
/// sim.reset(robotik::Seed{ 7 });
/// while (!sim.finished())
///     sim.step(robotik::Seconds(0.01));
/// for (robotik::Check const& check : sim.checks())
///     if (check.verdict != robotik::Check::Verdict::Passed) { ... }
/// @endcode
// ****************************************************************************
class Simulation
{
public:

    /// Why a run ended.
    enum class Outcome : std::uint8_t
    {
        Running,             ///< Not ended.
        ControllerSucceeded, ///< The tree, the planner or the mission succeeded.
        ControllerFailed,    ///< The tree, the planner or the mission failed.
        StopCondition,       ///< The @c stop_when condition holds.
        Timeout,             ///< The @c timeout elapsed.
        AssertionFailed,     ///< An assertion marked @c stop failed.
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
    /// @param p_mission Task plugin (skills, detectors, metrics); not owned.
    /// @throws std::runtime_error when the cell cannot be built.
    // -------------------------------------------------------------------------
    Simulation(compages::world::World& p_world,
               Scenario p_scenario,
               SceneView* p_view = nullptr,
               Mission* p_mission = nullptr);

    // -------------------------------------------------------------------------
    /// Starts a new episode: same seed, same episode.
    /// @param p_seed Seed of the episode (see @ref Cell::reset).
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
    [[nodiscard]] Cell& cell()
    {
        return m_cell;
    }

    /// @copydoc cell()
    [[nodiscard]] Cell const& cell() const
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

    // -------------------------------------------------------------------------
    /// Evaluates a condition on the metrics and the truth of the cell.
    /// @param p_condition E.g. @c "inside(red_cube, box)".
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

    /// The scenario being run.
    Scenario m_scenario;
    /// The cell built from it.
    Cell m_cell;
    /// Judges the assertions.
    AssertionEngine m_assertions;
    /// Why the run ended.
    Outcome m_outcome = Outcome::Running;
};

// ----------------------------------------------------------------------------
/// @param p_outcome Outcome to name.
/// @return For example "timeout" or "controller succeeded".
// ----------------------------------------------------------------------------
[[nodiscard]] char const* toString(Simulation::Outcome p_outcome);

} // namespace robotik
