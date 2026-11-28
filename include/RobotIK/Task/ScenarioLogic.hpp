// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file ScenarioLogic.hpp
/// The C++ part of a demo: what it adds to a generic @ref robotik::RoboticCell (its
/// skills, its detectors, its metrics, what it does at each step).
///
/// A scenario logic does not judge the run: it publishes numbers, and the assertions
/// of the scenario decide (see doc/reference/glossary.md: "Metric",
/// "Assertion").
///
/// @par Contract
/// - Promises: @c setup once, @c reset at each episode; after each physics
///   step @c act, @c simulate, then @c observe; @c measure publishes
///   numbers, never a verdict.
/// - Expects: @c act decides from the robot and its beliefs only (it gets no
///   @ref robotik::RoboticCell); @c observe reads the truth and commands nothing (it
///   gets a const cell); @c simulate changes the world only, never the
///   robot. The scenario logic does not keep entities between calls.
#pragma once

#include "RobotIK/Common/Random.hpp"
#include "RobotIK/Task/Metrics.hpp"
#include "RobotIK/Task/RobotContext.hpp"
#include "RobotIK/Task/Status.hpp"

#include "RobotIK/Common/Math.hpp"

namespace robotik
{

class SceneView;
class RoboticCell;

// ****************************************************************************
/// The part of a demo that only C++ can say: its skills, its detectors and
/// the numbers that measure it.
///
/// A scenario file describes the world, the controller and the verdict, but
/// it cannot say how a robot paints a stroke or what "a lap" is. The scenario logic
/// brings this code to a generic @ref RoboticCell, so that the core keeps no demo
/// object. Write one per scenario of a plugin (the table of
/// @c ScenarioPlugin), or give one to @ref robotik::StandaloneCell::open from a C++ program. A
/// program that runs only the built-in skills needs no scenario logic.
///
/// The cell calls @ref setup once, then for each episode @ref reset, and
/// after each physics step @ref act, @ref simulate and @ref observe. The run
/// calls @ref measure when it needs the numbers: assertions, reports, RL
/// rewards.
///
/// The three per-step methods split what a step used to mix, and the types
/// enforce the split. @ref act is the robot side: it gets a
/// @ref RobotContext, the same view as a skill, so it cannot read the true
/// pose of an object and the code runs unchanged on a real robot.
/// @ref observe is the judge side: it gets a const @ref RoboticCell, so it reads
/// the truth (for metrics) but cannot command anything. @ref simulate is the
/// world side, for a demo whose world changes by itself (a conveyor moves a
/// part, a brush leaves ink): it exists only in simulation. Every method has an
/// empty default, so a small demo stays small (HelloLogic overrides
/// three). The cell owns the world and the robots; the scenario logic owns only its
/// counters, and the cell keeps a pointer to it, never a copy.
///
/// A scenario logic does not judge the run: it publishes numbers, and the
/// assertions of the scenario decide. @ref status ends a run only when the
/// scenario has no behavior tree and no planner. Entities are found by id at
/// each call, never kept.
///
/// @code
/// class Walk final: public robotik::ScenarioLogic
/// {
///     void act(robotik::RobotContext& p_context, robotik::Seconds) override
///     {
///         m_policy.drive(p_context.robot);      // what the robot measures
///     }
///     void observe(robotik::RoboticCell const& p_cell, robotik::Seconds) override
///     {
///         m_lowest = std::min(m_lowest, p_cell.requirePose("base").position.z);
///     }
///     void measure(robotik::RoboticCell const&,
///                  robotik::Metrics& p_metrics) const override
///     {
///         p_metrics.set("walk.height_min", m_lowest);
///     }
/// };
/// @endcode
// ****************************************************************************
class ScenarioLogic
{
public:

    virtual ~ScenarioLogic() = default;

    // -------------------------------------------------------------------------
    /// Called once, after the scenario is built and before the first reset.
    /// Add the skills, the detectors and the extra world entities here.
    /// @param p_cell The cell running the scenario logic.
    /// @param p_view Rendering hooks; null for a headless run.
    // -------------------------------------------------------------------------
    virtual void setup(RoboticCell& /*p_cell*/, SceneView* /*p_view*/)
    {
        // Nothing to do by default.
    }

    // -------------------------------------------------------------------------
    /// Called at each episode start, after the robot and the objects are
    /// reset. Clear the counters of the scenario logic here.
    /// @param p_cell The cell running the scenario logic.
    /// @param p_seed Seed of the episode; derive random streams from it.
    // -------------------------------------------------------------------------
    virtual void reset(RoboticCell& /*p_cell*/, Seed /*p_seed*/)
    {
        // Nothing to do by default.
    }

    // -------------------------------------------------------------------------
    /// Robot side of a step: command the robot, update what the robot
    /// itself knows (odometry, a control law). Called after each physics
    /// step, with the context of the first robot (the others are in
    /// @c p_context.team).
    /// @param p_context Robot API, beliefs and time; no ground truth.
    /// @param p_dt Duration of the step.
    // -------------------------------------------------------------------------
    virtual void act(RobotContext& /*p_context*/, Seconds /*p_dt*/)
    {
        // Nothing to do by default.
    }

    // -------------------------------------------------------------------------
    /// World side of a step, in simulation only: what the world does by
    /// itself (a conveyor carries a part, ink lands on a canvas). Called after
    /// @ref act. It may read and change the entities; it must not command the
    /// robots nor request skills.
    /// @param p_cell The cell running the scenario logic.
    /// @param p_dt Duration of the step.
    // -------------------------------------------------------------------------
    virtual void simulate(RoboticCell& /*p_cell*/, Seconds /*p_dt*/)
    {
        // Nothing to do by default.
    }

    // -------------------------------------------------------------------------
    /// Judge side of a step: read the truth and accumulate the counters that
    /// @ref measure publishes (a minimum height, a time in contact). Called
    /// after @ref simulate.
    /// @param p_cell The cell, read only: no command is possible.
    /// @param p_dt Duration of the step.
    // -------------------------------------------------------------------------
    virtual void observe(RoboticCell const& /*p_cell*/, Seconds /*p_dt*/)
    {
        // Nothing to do by default.
    }

    // -------------------------------------------------------------------------
    /// Publishes the plugin metrics (called on demand). The scenario
    /// assertions read them; do not decide pass or fail here.
    /// @param p_cell The cell running the scenario logic.
    /// @param p_metrics Table receiving the values.
    // -------------------------------------------------------------------------
    virtual void measure(RoboticCell const& /*p_cell*/,
                         Metrics& /*p_metrics*/) const
    {
        // Nothing to do by default.
    }

    // -------------------------------------------------------------------------
    /// Outcome for scenario logics without behavior tree nor planner. Ignored
    /// otherwise. It is a decision of the robot, so it sees what @ref act
    /// sees.
    /// @param p_context Robot API, beliefs and time of the first robot.
    /// @return @ref Status::SUCCESS or @ref Status::FAILURE end the episode;
    ///   any other value lets it go on.
    // -------------------------------------------------------------------------
    [[nodiscard]] virtual Status
    status(RobotContext const& /*p_context*/) const
    {
        return Status::IDLE;
    }
};

} // namespace robotik
