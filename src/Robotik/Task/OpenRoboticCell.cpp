// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "Robotik/Task/OpenRoboticCell.hpp"

#include <stdexcept>

namespace robotik
{

Result<OpenedRoboticCell> openRoboticCell(RoboticCellDescription p_description,
                                          ScenarioLogic* p_logic)
{
    // A wrong description is a message, before anything is built.
    ROBOTIK_TRY(validate(p_description));
    // A model file that cannot be loaded still throws in the backends.
    try
    {
        OpenedRoboticCell opened;
        opened.world = std::make_unique<compages::world::World>();
        opened.cell = std::make_unique<RoboticCell>(
            *opened.world, std::move(p_description), nullptr, p_logic);
        return opened;
    }
    catch (std::exception const& error)
    {
        return failure(error.what());
    }
}

} // namespace robotik
