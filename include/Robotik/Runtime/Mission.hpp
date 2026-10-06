// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

//! @file Mission.hpp
//! @brief What a demo adds to a generic @ref Simulation: its skills, its
//! world, its success criteria and its metrics.
#pragma once

#include "Robotik/Math/Random.hpp"
#include "Robotik/Runtime/Metrics.hpp"
#include "Robotik/Runtime/Status.hpp"

#include "Compages/Core/Units.hpp"

namespace robotik
{

class SceneView;
class Simulation;

// ****************************************************************************
//! @brief Plugin of a @ref Simulation, provided by the application.
//!
//! The simulation builds what the scenario declares (robot, sensors,
//! actuators, objects, faults) and the generic skills (@c Home, @c Stop).
//! The mission adds the rest: task skills and behavior tree nodes, perception
//! detectors, extra world dressing, and the metrics the scenario asserts on.
//!
//! @code
//! class Patrol final: public robotik::Mission
//! {
//!     void setup(robotik::Simulation& p_sim, robotik::SceneView*) override
//!     {
//!         p_sim.skills().add<GoToSkill>(...);
//!     }
//!     void measure(robotik::Simulation const& p_sim,
//!                  robotik::Metrics& p_metrics) const override
//!     {
//!         p_metrics.set("laps", m_laps);
//!     }
//! };
//! @endcode
// ****************************************************************************
class Mission
{
public:

    virtual ~Mission() = default;

    // -------------------------------------------------------------------------
    //! @brief Called once, after the scenario is built and before the first
    //! reset. @p_view is null for a headless run.
    // -------------------------------------------------------------------------
    virtual void setup(Simulation& /*p_simulation*/, SceneView* /*p_view*/) {}

    // -------------------------------------------------------------------------
    //! @brief Called at each episode start, after the robot and the objects
    //! are reset. Derive random streams from @p_seed.
    // -------------------------------------------------------------------------
    virtual void reset(Simulation& /*p_simulation*/, Seed /*p_seed*/) {}

    // -------------------------------------------------------------------------
    //! @brief Called after each robot step.
    // -------------------------------------------------------------------------
    virtual void step(Simulation& /*p_simulation*/, Seconds /*p_dt*/) {}

    // -------------------------------------------------------------------------
    //! @brief Publishes the mission metrics (called on demand).
    // -------------------------------------------------------------------------
    virtual void measure(Simulation const& /*p_simulation*/,
                         Metrics& /*p_metrics*/) const
    {
    }

    // -------------------------------------------------------------------------
    //! @brief Outcome for missions without behavior tree: @ref Status::SUCCESS
    //! or @ref Status::FAILURE end the episode.
    // -------------------------------------------------------------------------
    [[nodiscard]] virtual Status status(Simulation const& /*p_simulation*/) const
    {
        return Status::IDLE;
    }
};

} // namespace robotik
