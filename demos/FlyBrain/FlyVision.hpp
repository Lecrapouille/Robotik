// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

// @file FlyVision.hpp
// @brief Three synthetic eyes: left, centre and right.
#pragma once

#include "FlyTypes.hpp"

#include "Robotik/Math/Random.hpp"

#include <span>

// ****************************************************************************
//! @brief One sample of the three eyes.
//!
//! Each value is 1 when an obstacle touches the eye and 0 when the ray
//! reaches its end without a hit. @c origins[i] is where beam i leaves the
//! head (left, centre, right). @c ends[i] is the hit, or the end of the
//! range on a miss. Index 0 is the left eye.
// ****************************************************************************
struct FlyVision
{
    //!< Left eye, in [0, 1].
    float left = 0.0f;
    //!< Centre eye, in [0, 1].
    float center = 0.0f;
    //!< Right eye, in [0, 1].
    float right = 0.0f;
    //!< Distance along the nearest ray that hit (SI: m). The range, if none did.
    float distance = 0.0f;
    //!< Start of each beam, on the head (SI: m, z up).
    robotik::Vector3 origins[3]{};
    //!< End of each beam (SI: m, z up).
    robotik::Vector3 ends[3]{};
};

// ------------------------------------------------------------------------
//! @brief Casts the three eye rays.
//!
//! @p_neck_yaw is the head yaw in the thorax (SI: rad). The beams leave the
//! eye meshes, not the thorax centre. @p_noise, when not null and when
//! @p_sigma is positive, adds gaussian noise to each eye and clamps it to
//! [0, 1].
// ------------------------------------------------------------------------
[[nodiscard]] FlyVision senseFly(FlyPlant const& p_plant,
                                 std::span<FlyBox const> p_obstacles,
                                 robotik::Random* p_noise,
                                 double p_sigma,
                                 double p_neck_yaw = 0.0);
