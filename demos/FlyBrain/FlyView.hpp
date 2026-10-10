// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

// @file FlyView.hpp
// @brief Compages window for one fly.
//
// The brain never calls this. The chase camera follows the thorax. Each eye
// link carries a camera, drawn in a corner of the window.
#pragma once

#include "FlyEnvironment.hpp"

#include <memory>

// ****************************************************************************
//! @brief One window: the URDF, the arena, the eye beams and the eye cameras.
// ****************************************************************************
class FlyView
{
public:

    // ------------------------------------------------------------------------
    //! @brief Keeps the scenario. The window opens in @ref open.
    // ------------------------------------------------------------------------
    explicit FlyView(FlyScenario const& p_scenario);

    // ------------------------------------------------------------------------
    //! @brief Closes the window if @ref open succeeded.
    // ------------------------------------------------------------------------
    ~FlyView();

    FlyView(FlyView const&) = delete;
    FlyView& operator=(FlyView const&) = delete;

    // ------------------------------------------------------------------------
    //! @brief Creates the OpenGL window, the URDF and the eye cameras.
    //! @return false when the window could not be opened.
    // ------------------------------------------------------------------------
    bool open();

    // ------------------------------------------------------------------------
    //! @brief Poses the URDF, the rays and the cameras, then draws one frame.
    //! @return false when the user has closed the window or pressed Escape.
    // ------------------------------------------------------------------------
    bool frame(FlyEnvironment const& p_environment);

private:

    //!< GPU objects. Defined in the cpp so this header stays free of GLFW.
    struct Gpu;

    //!< Scenario the meshes were built from.
    FlyScenario m_scenario;
    //!< Empty until @ref open.
    std::unique_ptr<Gpu> m_gpu;
};
