// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file Queries.hpp
/// ECS lookup of scenario objects by name.
#pragma once

#include "Robotik/ECS/ObjectComponents.hpp"

#include "Compages/World/Entity.hpp"
#include "Compages/World/World.hpp"

#include <string_view>

namespace robotik
{

// -------------------------------------------------------------------------
/// Finds the entity with @ref ecs::Shape matching @p_name.
///
/// The search walks every shape of the world, so call it when needed rather
/// than keeping the entity: a plugin does not own world entities and must
/// find them by id each time.
///
/// @code
/// compages::world::Entity box = robotik::findObject(world, "red_cube");
/// if (box)
///     auto const& shape = box.get<robotik::ecs::Shape>();
/// @endcode
///
/// @param p_world World to search.
/// @param p_name Object name from the scenario.
/// @return Matching entity, or an invalid entity if not found.
// -------------------------------------------------------------------------
[[nodiscard]] inline compages::world::Entity
findObject(compages::world::World& p_world, std::string_view p_name)
{
    compages::world::Entity found;
    p_world.each<ecs::Shape>(
        [&found, p_name](compages::world::Entity p_entity, ecs::Shape& p_object)
        {
            if (!found && p_object.name == p_name)
            {
                found = p_entity;
            }
        });
    return found;
}

} // namespace robotik
