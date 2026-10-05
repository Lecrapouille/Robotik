// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

// @file Queries.hpp
// @brief ECS lookup of scenario objects by name.
#pragma once

#include "Robotik/ECS/ObjectComponents.hpp"

#include "Compages/World/Entity.hpp"
#include "Compages/World/World.hpp"

#include <string_view>

namespace robotik
{

// -------------------------------------------------------------------------
//! @brief Finds the entity with @ref ecs::SceneObject matching @p_name.
//! @param p_world World to search.
//! @param p_name Object name from the scenario.
//! @return Matching entity, or invalid if not found.
// -------------------------------------------------------------------------
[[nodiscard]] inline compages::world::Entity
findObject(compages::world::World& p_world, std::string_view p_name)
{
    compages::world::Entity found;
    p_world.each<ecs::SceneObject>(
        [&found, p_name](compages::world::Entity p_entity,
                         ecs::SceneObject& p_object)
        {
            if (!found && p_object.name == p_name)
            {
                found = p_entity;
            }
        });
    return found;
}

} // namespace robotik
