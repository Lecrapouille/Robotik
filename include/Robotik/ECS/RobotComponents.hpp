// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

// @file RobotComponents.hpp
// @brief ECS markers of a loaded robot root entity.
#pragma once

#include <filesystem>
#include <string>

namespace robotik::ecs
{

// ****************************************************************************
//! @brief Marks the URDF root entity as the robot instance.
// ****************************************************************************
struct RobotTag
{
    //!< Reserved lifecycle flag (always true for loaded robots).
    bool alive = true;
};

// ****************************************************************************
//! @brief Human-readable robot id and source model path.
// ****************************************************************************
struct RobotIdentity
{
    //!< Name from the URDF @c robot element.
    std::string name;
    //!< URDF file.
    std::filesystem::path model_path;
};

} // namespace robotik::ecs
