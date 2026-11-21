// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "Robotik/Task/OpenCell.hpp"

#include <stdexcept>

namespace robotik
{

Result<OpenedCell> openCell(CellDescription p_description, Mission* p_mission)
{
    // The border of this builder: whatever stops the cell becomes a message.
    try
    {
        OpenedCell opened;
        opened.world = std::make_unique<compages::world::World>();
        opened.cell = std::make_unique<Cell>(
            *opened.world, std::move(p_description), nullptr, p_mission);
        return opened;
    }
    catch (std::exception const& error)
    {
        return failure(error.what());
    }
}

} // namespace robotik
