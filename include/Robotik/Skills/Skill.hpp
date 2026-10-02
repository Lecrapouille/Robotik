// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

//! @file Skill.hpp
//! @brief Abstract unit of robot behavior ticked from code or behavior trees.
#pragma once

#include "Compages/Core/Units.hpp"
#include "Robotik/Runtime/Status.hpp"

namespace robotik
{

struct RobotContext;

// ****************************************************************************
//! @brief One-shot or continuous action that writes @ref ecs::JointCommand.
//!
//! Skills read state through @ref RobotContext and ECS; they do not call MuJoCo
//! or drivers directly. Register instances with @ref registerSkill for BT use.
//!
//! @example
//! @code
//! class WaveSkill : public robotik::Skill {
//! public:
//!     robotik::Status tick(robotik::RobotContext& ctx, Seconds dt) override {
//!         // set joint commands...
//!         return robotik::Status::RUNNING;
//!     }
//! };
//! @endcode
// ****************************************************************************
class Skill
{
public:

    virtual ~Skill() = default;

    // -------------------------------------------------------------------------
    //! @brief Clears internal planning state before a new BT action run.
    //!
    //! Default implementation does nothing.
    // -------------------------------------------------------------------------
    virtual void reset()
    {
        /* no-op */
    }

    // -------------------------------------------------------------------------
    //! @brief Advances the skill by one simulation step.
    //! @param p_context World, backends, time, and dt.
    //! @param p_dt Step duration (often equals @c p_context.dt).
    //! @return @ref Status::RUNNING until the goal is met or failed.
    // -------------------------------------------------------------------------
    virtual Status tick(RobotContext& p_context, Seconds p_dt) = 0;
};

} // namespace robotik
