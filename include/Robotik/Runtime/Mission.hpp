// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file Mission.hpp
/// What a demo adds to a generic @ref Simulation: its skills, its world, its
/// success criteria and its metrics.
#pragma once

#include "Robotik/Math/Random.hpp"
#include "Robotik/Runtime/Metrics.hpp"
#include "Robotik/Runtime/Status.hpp"

#include "Robotik/Math/Si.hpp"

namespace robotik
{

class SceneView;
class Simulation;

// ****************************************************************************
/// Plugin of a @ref Simulation, provided by the application.
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
///     void setup(robotik::Simulation& p_sim, robotik::SceneView*) override
///     {
///         p_sim.skills().add<GoToSkill>(...);
///     }
///     void measure(robotik::Simulation const& p_sim,
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
    /// @param p_simulation The simulation running the mission.
    /// @param p_view Rendering hooks; null for a headless run.
    // -------------------------------------------------------------------------
    virtual void setup(Simulation& /*p_simulation*/, SceneView* /*p_view*/) {}

    // -------------------------------------------------------------------------
    /// Called at each episode start, after the robot and the objects are
    /// reset. Clear the counters of the mission here.
    /// @param p_simulation The simulation running the mission.
    /// @param p_seed Seed of the episode; derive random streams from it.
    // -------------------------------------------------------------------------
    virtual void reset(Simulation& /*p_simulation*/, Seed /*p_seed*/) {}

    // -------------------------------------------------------------------------
    /// Called after each robot step. Update the counters here.
    /// @param p_simulation The simulation running the mission.
    /// @param p_dt Duration of the step.
    // -------------------------------------------------------------------------
    virtual void step(Simulation& /*p_simulation*/, Seconds /*p_dt*/) {}

    // -------------------------------------------------------------------------
    /// Publishes the mission metrics (called on demand). The scenario
    /// assertions read them; do not decide pass or fail here.
    /// @param p_simulation The simulation running the mission.
    /// @param p_metrics Table receiving the values.
    // -------------------------------------------------------------------------
    virtual void measure(Simulation const& /*p_simulation*/,
                         Metrics& /*p_metrics*/) const
    {
    }

    // -------------------------------------------------------------------------
    /// Outcome for missions without behavior tree nor planner. Ignored
    /// otherwise.
    /// @param p_simulation The simulation running the mission.
    /// @return @ref Status::SUCCESS or @ref Status::FAILURE end the episode;
    ///   any other value lets it go on.
    // -------------------------------------------------------------------------
    [[nodiscard]] virtual Status
    status(Simulation const& /*p_simulation*/) const
    {
        return Status::IDLE;
    }
};

} // namespace robotik
