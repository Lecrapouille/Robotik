// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file OpenCell.hpp
/// The shortest way to get a robot cell: one call, no world to create.
///
/// A @ref robotik::Cell lives in an ECS world (Compages) that the
/// simulator shares with its rendering. A program that only wants robots,
/// objects and skills should not have to know it: @ref robotik::openCell
/// creates the world and keeps it with the cell.
///
/// @code
/// robotik::EntitySpec arm;
/// arm.id = "arm";
/// arm.type = robotik::EntitySpec::Type::Robot;
/// arm.model = "robot_6axis.urdf";
/// robotik::CellDescription description;
/// description.world.entities = { arm };
/// auto cell = robotik::openCell(std::move(description));
/// if (!cell)
///     return fail(cell.error());
/// auto const raise = cell.value()->skills().add<robotik::MoveJointSkill>(
///     { .name = "Raise" }, "joint2", 0.5);
/// cell.value()->run(raise, robotik::Seconds(10.0));
/// @endcode
///
/// This is the second layer of RobotIK: robots doing a job. The layer above
/// (Package) reads the same description from YAML files.
///
/// @par Contract
/// - Promises: @c openCell returns a @c Result: a world that cannot be built
///   (unknown model, bad goal) is a message, not an exception.
/// - Expects: a description whose files (URDF, PDDL) exist.
#pragma once

#include "Robotik/Common/Result.hpp"
#include "Robotik/Task/Cell.hpp"

#include "Compages/World/World.hpp"

#include <memory>

namespace robotik
{

// ****************************************************************************
/// A cell with the world it lives in. Use it like a pointer to the cell.
///
/// The world must live longer than the cell: the members are declared in
/// this order, so the cell is destroyed first. Move it, do not copy it.
// ****************************************************************************
struct OpenedCell
{
    /// ECS world holding the entities.
    std::unique_ptr<compages::world::World> world;
    /// The cell.
    std::unique_ptr<Cell> cell;

    /// @return The cell.
    Cell* operator->() const
    {
        return cell.get();
    }

    /// @return The cell.
    Cell& operator*() const
    {
        return *cell;
    }
};

// ----------------------------------------------------------------------------
/// Builds a cell and its world, headless.
/// @param p_description Robots, objects, controller.
/// @param p_mission Task plugin (skills, detectors, metrics), or null; not
///   owned.
/// @return The cell, or why it could not be built.
// ----------------------------------------------------------------------------
[[nodiscard]] Result<OpenedCell> openCell(CellDescription p_description,
                                          Mission* p_mission = nullptr);

} // namespace robotik
