// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file ShapeOf.hpp
/// @brief Private to the runtime: the shape component of a world entity.
#pragma once

#include "Robotik/ECS/ObjectComponents.hpp"
#include "Robotik/Scenario/World.hpp"

namespace robotik
{

/// @brief Shape of @p_spec, which must have a @c shape: section.
[[nodiscard]] inline ecs::Shape shapeOf(EntitySpec const& p_spec)
{
    ecs::Shape object;
    object.name = p_spec.id;
    object.type = p_spec.shape->type == ShapeSpec::Type::Container ? ecs::Shape::Type::CONTAINER
                                                                   : ecs::Shape::Type::BOX;
    object.size = { Length(p_spec.shape->size.x),
                    Length(p_spec.shape->size.y),
                    Length(p_spec.shape->size.z) };
    object.color = p_spec.shape->color;
    return object;
}

} // namespace robotik
