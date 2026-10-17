// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

//! @file GraspSystem.hpp
//! @brief Simulated suction: what a @ref VacuumGripper holds.
//!
//! Plays the part of the vacuum sensor and of object physics in simulation:
//! suction near the top of a box prop attaches it, no suction drops it onto
//! the floor of the container below (props have no dynamics). Props live in
//! the world frame; bodies of the physics backend and decorations are left
//! alone.
#pragma once

namespace robotik
{

class Robot;

// ****************************************************************************
//! @brief Attaches, carries and drops @ref ecs::Shape entities.
// ****************************************************************************
class GraspSystem
{
public:

    //! @brief Updates every vacuum gripper of @p_robot after a robot step.
    void update(Robot& p_robot) const;
};

} // namespace robotik
