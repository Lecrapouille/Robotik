// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

//! @file RobotContext.hpp
//! @brief Everything a skill may use during one tick.
#pragma once

#include "Compages/Core/Units.hpp"

namespace robotik
{

class Robot;
class WorldModel;

// ****************************************************************************
//! @brief Per-tick view handed to skills: the robot API and the beliefs.
//!
//! There is no simulator in here: a skill cannot tell MuJoCo from hardware.
// ****************************************************************************
struct RobotContext
{
    //!< Joints, sensors, actuators, resources and kinematics.
    Robot& robot;
    //!< Beliefs about the objects around the robot.
    WorldModel& world_model;
    //!< Robot clock at the start of the tick.
    Seconds time{};
    //!< Duration of the tick.
    Seconds dt{};
};

} // namespace robotik
