// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

// @file FlyController.hpp
// @brief Maps a fly action onto the thorax and a posture.
//
// It does not know why the action was chosen.
#pragma once

#include "FlyTypes.hpp"

// ****************************************************************************
//! @brief Kinematic body of the fly.
//!
//! Turns a @ref FlyAction into thrust, yaw and lift, and into joint targets
//! for the URDF. There is no aerodynamics and no foot adhesion.
// ****************************************************************************
class FlyController
{
public:

    // ------------------------------------------------------------------------
    //! @brief Wings back to the start of a stroke, speeds left to the plant.
    // ------------------------------------------------------------------------
    void reset();

    // ------------------------------------------------------------------------
    //! @brief Integrates @p_action into @p_plant over @p_dt seconds and
    //! refreshes @ref posture.
    // ------------------------------------------------------------------------
    void apply(FlyAction const& p_action, FlyPlant& p_plant, double p_dt);

    // ------------------------------------------------------------------------
    //! @brief Joint targets written by the last @ref apply or @ref reset.
    // ------------------------------------------------------------------------
    [[nodiscard]] FlyPosture const& posture() const
    {
        return m_posture;
    }

private:

    //!< Last joint targets.
    FlyPosture m_posture;
    //!< Wing-stroke phase (SI: rad).
    double m_phase = 0.0;
};
