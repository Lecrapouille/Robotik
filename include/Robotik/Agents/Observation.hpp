// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

//! @file Observation.hpp
//! @brief What an agent is allowed to know at one instant.
//!
//! A flat vector, same layout an @ref Environment writes into its observation
//! buffer. The agent never receives a camera image, a texture or a simulator
//! handle.
#pragma once

#include "Compages/Core/Units.hpp"

#include <vector>

namespace robotik
{

// ****************************************************************************
//! @brief Timestamped sensor vector handed to an @ref Agent.
// ****************************************************************************
struct Observation
{
    Seconds timestamp{};
    std::vector<float> values;
};

} // namespace robotik
