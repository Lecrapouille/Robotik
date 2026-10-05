// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

//! @file Skill.hpp
//! @brief Unit of robot behavior and its declarative description.
#pragma once

#include "Robotik/Runtime/Resources.hpp"
#include "Robotik/Runtime/Status.hpp"

#include "Compages/Core/Units.hpp"

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace robotik
{

struct RobotContext;

//! @brief Higher runs first and may preempt lower (e.g. stop = 1000).
using Priority = std::int32_t;

// ****************************************************************************
//! @brief Condition that must hold before a skill starts.
// ****************************************************************************
struct Precondition
{
    //!< Human-readable text shown when the skill is blocked.
    std::string text;
    std::function<bool(RobotContext const&)> holds;
};

// ****************************************************************************
//! @brief What a skill needs and how the scheduler should treat it.
//!
//! @code
//! robotik::SkillDescription pick{
//!     .name = "Pick(red_cube)",
//!     .resources = { resources.require("arm"),
//!                    resources.require("gripper"),
//!                    resources.require("camera", robotik::Access::Shared) },
//!     .priority = 100 };
//! @endcode
// ****************************************************************************
struct SkillDescription
{
    //!< Unique name, also the behavior tree action name.
    std::string name{};
    //!< Resources reserved for the whole run.
    std::vector<ResourceRequirement> resources{};
    Priority priority = 0;
    //!< Waits while a resource is busy or a precondition is false, instead of
    //!< failing at once.
    bool wait = true;
    //!< False forbids preemption by a higher priority skill.
    bool cancellable = true;
    std::vector<Precondition> preconditions{};
};

// ****************************************************************************
//! @brief One-shot or continuous robot action.
//!
//! Skills read and command the robot through @ref RobotContext only, so the
//! same code drives MuJoCo, hardware or an RL environment. They are executed
//! by the @ref SkillScheduler, which reserves their resources first.
//!
//! @code
//! class WaveSkill final : public robotik::Skill {
//! public:
//!     robotik::Status tick(robotik::RobotContext& p_context, Seconds) override
//!     {
//!         // command joints through p_context.robot.joints() ...
//!         return robotik::Status::RUNNING;
//!     }
//!     void cancel(robotik::RobotContext& p_context) override
//!     {
//!         p_context.robot.joints().hold(); // stop where we are
//!     }
//! };
//! @endcode
// ****************************************************************************
class Skill
{
public:

    virtual ~Skill() = default;

    // -------------------------------------------------------------------------
    //! @brief Clears the run state; called each time the skill starts.
    // -------------------------------------------------------------------------
    virtual void reset()
    {
        /* no-op */
    }

    // -------------------------------------------------------------------------
    //! @brief Advances the skill by one step.
    //! @return @ref Status::RUNNING until the goal is met or failed.
    // -------------------------------------------------------------------------
    virtual Status tick(RobotContext& p_context, Seconds p_dt) = 0;

    // -------------------------------------------------------------------------
    //! @brief Cooperative stop (cancel, preemption, lost resource): bring the
    //! commanded hardware to a safe state. Composite skills forward it to
    //! their children. The skill is not ticked again for this run.
    // -------------------------------------------------------------------------
    virtual void cancel(RobotContext& /*p_context*/)
    {
        /* no-op */
    }
};

} // namespace robotik
