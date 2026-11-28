// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file StandaloneCell.hpp
/// The shortest way to get a robot cell: one call, no world to create.
///
/// A @ref robotik::RoboticCell lives in an ECS world (Compages) that the
/// simulator shares with its rendering. A program that only wants robots,
/// objects and skills should not have to know it: @ref robotik::StandaloneCell::open
/// creates the world and keeps it with the cell.
///
/// @code
/// robotik::EntitySpec arm;
/// arm.id = "arm";
/// arm.type = robotik::EntitySpec::Type::Robot;
/// arm.model = "robot_6axis.urdf";
/// robotik::RoboticCellDescription description;
/// description.world.entities = { arm };
/// auto cell = robotik::StandaloneCell::open(std::move(description));
/// if (!cell)
///     return fail(cell.error());
/// auto const raise = cell.value().cell().skills().add<robotik::MoveJointSkill>(
///     { .name = "Raise" }, "joint2", 0.5);
/// cell.value().cell().run(raise, robotik::Seconds(10.0));
/// @endcode
///
/// This is the second layer of RobotIK: robots doing a job, next to
/// @ref robotik::StandaloneRobot (one robot) and
/// @ref robotik::StandaloneSimulation (a scenario file and its verdict). The
/// three have the same shape: @c open returns a @c Result, the world is
/// private and destroyed last, @c step makes time pass. The scenario logic
/// is given in C++ and not owned: loading it from a plugin library is a
/// matter of packages, so it is done one layer above, by
/// @ref robotik::StandaloneSimulation::openPlugin.
///
/// @par Contract
/// - Promises: @c StandaloneCell::open returns a @c Result: a world that cannot be built
///   (unknown model, bad goal) is a message, not an exception. The world
///   lives longer than the cell, whatever the caller does: both are private.
/// - Expects: a description whose files (URDF, PDDL) exist.
#pragma once

#include "RobotIK/Common/Result.hpp"
#include "RobotIK/Task/RoboticCell.hpp"

#include "Compages/World/World.hpp"

#include <memory>

namespace robotik
{

// ****************************************************************************
/// A robot cell alone, with the ECS world it lives in, headless: for a C++
/// program without YAML files, a test, an RL environment.
///
/// Why a class: the cell keeps a reference to the world, so the world must
/// be destroyed after it. Both are private, the world is declared first
/// (so destroyed last), and the class moves but is not assigned (an
/// assignment would destroy the old world before the old cell).
///
/// @code
/// auto opened = robotik::StandaloneCell::open(std::move(description));
/// robotik::RoboticCell& cell = opened.value().cell();
/// cell.step(robotik::Seconds(0.01));
/// @endcode
// ****************************************************************************
class StandaloneCell final
{
public:

    // -------------------------------------------------------------------------
    /// Builds a cell and its world, headless.
    /// @param p_description Robots, objects, controller.
    /// @param p_logic Scenario logic (skills, detectors, metrics), or null;
    ///   not owned: it must outlive the cell.
    /// @return The cell, or why it could not be built.
    // -------------------------------------------------------------------------
    [[nodiscard]] static Result<StandaloneCell>
    open(RoboticCellDescription p_description,
         ScenarioLogic* p_logic = nullptr);

    // -------------------------------------------------------------------------
    /// One step of the cell: the controller, the physics, the sensors, the
    /// scenario logic (see @ref RoboticCell::step).
    // -------------------------------------------------------------------------
    void step(Seconds p_dt)
    {
        m_cell->step(p_dt);
    }

    StandaloneCell(StandaloneCell&&) = default;
    StandaloneCell& operator=(StandaloneCell&&) = delete;
    StandaloneCell(StandaloneCell const&) = delete;
    StandaloneCell& operator=(StandaloneCell const&) = delete;
    ~StandaloneCell();

    /// @return The cell: robots, objects, physics, controller, beliefs.
    [[nodiscard]] RoboticCell& cell()
    {
        return *m_cell;
    }

    /// @copydoc cell()
    [[nodiscard]] RoboticCell const& cell() const
    {
        return *m_cell;
    }

    /// @return The ECS world of the cell (its entities).
    [[nodiscard]] compages::world::World& getWorld()
    {
        return *m_world;
    }

private:

    StandaloneCell();

    /// ECS world holding the entities; declared first, destroyed last.
    std::unique_ptr<compages::world::World> m_world;
    /// The cell.
    std::unique_ptr<RoboticCell> m_cell;
};

} // namespace robotik
