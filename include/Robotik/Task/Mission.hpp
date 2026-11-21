// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file Mission.hpp
/// The C++ part of a demo: what it adds to a generic @ref robotik::Cell (its
/// skills, its detectors, its metrics, what it does at each step).
///
/// A mission does not judge the run: it publishes numbers, and the assertions
/// of the scenario decide (see doc/reference/glossary.md: "Metric",
/// "Assertion").
///
/// @par Contract
/// - Promises: @c setup once, @c reset at each episode, @c step after each
///   physics step; @c measure publishes numbers, never a verdict.
/// - Expects: the mission reads beliefs to decide and the truth only to
///   measure; it does not keep entities between calls.
#pragma once

#include "Robotik/Common/Random.hpp"
#include "Robotik/Task/Metrics.hpp"
#include "Robotik/Task/Status.hpp"

#include "Robotik/Common/Math.hpp"

namespace robotik
{

class SceneView;
class Cell;

// ****************************************************************************
/// The part of a demo that only C++ can say: its skills, its detectors and
/// the numbers that measure it.
///
/// A scenario file describes the world, the controller and the verdict, but
/// it cannot say how a robot paints a stroke or what "a lap" is. The mission
/// brings this code to a generic @ref Cell, so that the core keeps no demo
/// object. Write one per scenario of a plugin (the table of
/// @c MissionPlugin), or give one to @ref openCell from a C++ program. A
/// program that runs only the built-in skills needs no mission.
///
/// The cell calls @ref setup once, then for each episode @ref reset and one
/// @ref step after each physics step. The run calls @ref measure when it
/// needs the numbers: assertions, reports, RL rewards. Every method has an
/// empty default, so a small demo stays small (HelloMission overrides
/// three). The cell owns the world and the robots; the mission owns only its
/// counters, and the cell keeps a pointer to it, never a copy.
///
/// A mission does not judge the run: it publishes numbers, and the
/// assertions of the scenario decide. @ref status ends a run only when the
/// scenario has no behavior tree and no planner. Entities are found by id at
/// each call, never kept.
///
/// @code
/// class Patrol final: public robotik::Mission
/// {
///     void setup(robotik::Cell& p_cell, robotik::SceneView*) override
///     {
///         p_cell.skills().add<GoToSkill>(...);
///     }
///     void measure(robotik::Cell const& p_cell,
///                  robotik::Metrics& p_metrics) const override
///     {
///         p_metrics.set("laps", m_laps);
///     }
/// };
/// @endcode
// ****************************************************************************
class Mission
{
public:

    virtual ~Mission() = default;

    // -------------------------------------------------------------------------
    /// Called once, after the scenario is built and before the first reset.
    /// Add the skills, the detectors and the extra world entities here.
    /// @param p_cell The cell running the mission.
    /// @param p_view Rendering hooks; null for a headless run.
    // -------------------------------------------------------------------------
    virtual void setup(Cell& /*p_cell*/, SceneView* /*p_view*/)
    {
        // Nothing to do by default.
    }

    // -------------------------------------------------------------------------
    /// Called at each episode start, after the robot and the objects are
    /// reset. Clear the counters of the mission here.
    /// @param p_cell The cell running the mission.
    /// @param p_seed Seed of the episode; derive random streams from it.
    // -------------------------------------------------------------------------
    virtual void reset(Cell& /*p_cell*/, Seed /*p_seed*/)
    {
        // Nothing to do by default.
    }

    // -------------------------------------------------------------------------
    /// Called after each robot step. Update the counters here.
    /// @param p_cell The cell running the mission.
    /// @param p_dt Duration of the step.
    // -------------------------------------------------------------------------
    virtual void step(Cell& /*p_cell*/, Seconds /*p_dt*/)
    {
        // Nothing to do by default.
    }

    // -------------------------------------------------------------------------
    /// Publishes the mission metrics (called on demand). The scenario
    /// assertions read them; do not decide pass or fail here.
    /// @param p_cell The cell running the mission.
    /// @param p_metrics Table receiving the values.
    // -------------------------------------------------------------------------
    virtual void measure(Cell const& /*p_cell*/,
                         Metrics& /*p_metrics*/) const
    {
        // Nothing to do by default.
    }

    // -------------------------------------------------------------------------
    /// Outcome for missions without behavior tree nor planner. Ignored
    /// otherwise.
    /// @param p_cell The cell running the mission.
    /// @return @ref Status::SUCCESS or @ref Status::FAILURE end the episode;
    ///   any other value lets it go on.
    // -------------------------------------------------------------------------
    [[nodiscard]] virtual Status
    status(Cell const& /*p_cell*/) const
    {
        return Status::IDLE;
    }
};

} // namespace robotik
