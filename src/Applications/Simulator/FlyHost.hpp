// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

// @file FlyHost.hpp
// @brief Hosts the fly closed loop inside the simulator window.
#pragma once

#include <cstdint>

struct App;

// ------------------------------------------------------------------------
//! @brief Builds the arena in the scene @ref App::load already created.
//!
//! Throws on a bad scenario, a missing URDF or a failed scene prepare.
// ------------------------------------------------------------------------
void loadFly(App& p_app);

// ------------------------------------------------------------------------
//! @brief Restarts the episode at @p_seed and poses the fly where it starts.
// ------------------------------------------------------------------------
void resetFly(App& p_app, std::uint64_t p_seed);

// ------------------------------------------------------------------------
//! @brief One brain and environment step, then the URDF and the eye beams.
// ------------------------------------------------------------------------
void stepFly(App& p_app);

// ------------------------------------------------------------------------
//! @brief Draws both eye cameras into @ref FlyWatch::eye_picture.
//!
//! Call after the world update, so the pictures follow the head.
// ------------------------------------------------------------------------
void renderFlyEyes(App& p_app);

//! Edge list extracted by external/compilation. Gitignored.
inline constexpr char const* FLYWIRE_EDGES = "data/flywire/edges.csv";

//! Role file: which row of the completeness table fills each sensor and motor.
inline constexpr char const* FLYWIRE_BINDING = "data/flywire/binding.txt";

// ------------------------------------------------------------------------
//! @brief Switches the fly between the 13-neuron circuit and the connectome.
//!
//! @p_connectome loads @ref FLYWIRE_EDGES and @ref FLYWIRE_BINDING, then
//! restarts the episode. On failure the previous brain stays and @ref App::error
//! explains why.
// ------------------------------------------------------------------------
void selectFlyBrain(App& p_app, bool p_connectome);
