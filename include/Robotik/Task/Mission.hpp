// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file Mission.hpp
/// What a demo adds to a generic @ref Cell: its skills, its world, its
/// success criteria and its metrics.
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
/// Plugin of a @ref Cell, provided by the application.
///
/// The simulation builds what the scenario declares (robot, sensors,
/// actuators, objects, faults) and the generic skills (@c Home, @c Stop).
/// The mission adds the rest: task skills and behavior tree nodes, perception
/// detectors, extra world dressing, and the metrics the scenario asserts on.
///
/// Every method has an empty default: override only what you need. The order
/// of calls is @ref setup once, then for each episode @ref reset and many
/// @ref step.
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
    { /* nothing to do by default */
    }

    // -------------------------------------------------------------------------
    /// Called at each episode start, after the robot and the objects are
    /// reset. Clear the counters of the mission here.
    /// @param p_cell The cell running the mission.
    /// @param p_seed Seed of the episode; derive random streams from it.
    // -------------------------------------------------------------------------
    virtual void reset(Cell& /*p_cell*/, Seed /*p_seed*/)
    { /* nothing to do by default */
    }

    // -------------------------------------------------------------------------
    /// Called after each robot step. Update the counters here.
    /// @param p_cell The cell running the mission.
    /// @param p_dt Duration of the step.
    // -------------------------------------------------------------------------
    virtual void step(Cell& /*p_cell*/, Seconds /*p_dt*/)
    { /* nothing to do by default */
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
