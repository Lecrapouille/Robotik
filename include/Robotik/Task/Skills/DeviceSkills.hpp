// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file DeviceSkills.hpp
/// Skills of the devices of the cell (Task/CellDevices.hpp): run a belt.
///
/// The cell adds @c <belt>.Run for each conveyor belt of the world. The
/// skill runs while it is requested: an action of a GRAFCET step (the belt
/// runs while the step is active), or a node of a behavior tree under a
/// parallel or a timeout.
///
/// @par Contract
/// - Promises: the belt runs from the first tick until the skill is
///   cancelled or the resource of the belt fails; it never succeeds alone.
/// - Expects: the belt outlives the skill (both belong to the cell).
#pragma once

#include "Robotik/Task/CellDevices.hpp"
#include "Robotik/Task/Skills/Skill.hpp"

namespace robotik
{

// ****************************************************************************
/// Runs a conveyor belt until it is cancelled.
///
/// @code
/// scheduler.add<robotik::RunBeltSkill>(
///     { .name = "feeder.Run", .resources = { resources.require("feeder") } }, belt);
/// @endcode
// ****************************************************************************
class RunBeltSkill final: public Skill
{
public:

    /// @param p_belt Belt to run.
    explicit RunBeltSkill(ConveyorBelt& p_belt) : m_belt(p_belt) {}

    /// Runs the belt; RUNNING until cancelled.
    Status tick(RobotContext& p_context, Seconds p_dt) override;
    /// Stops the belt.
    void cancel(RobotContext& p_context) override;

private:

    ConveyorBelt& m_belt;
};

} // namespace robotik
