// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

//! @file Action.hpp
//! @brief What an agent decided. A controller turns it into motor commands.
#pragma once

#include <vector>

namespace robotik
{

// ****************************************************************************
//! @brief Flat command vector produced by an @ref Agent.
// ****************************************************************************
struct Action
{
    std::vector<float> values;
};

} // namespace robotik
