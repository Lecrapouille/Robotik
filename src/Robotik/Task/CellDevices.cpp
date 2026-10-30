// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "Robotik/Task/CellDevices.hpp"

#include <cmath>

namespace robotik
{

ConveyorBelt::ConveyorBelt(std::string p_name, std::string p_entity, Vector3 p_direction,
                           LinearVelocity p_speed)
    : CellDevice(std::move(p_name), std::move(p_entity)), m_speed(p_speed)
{
    double const norm = std::hypot(p_direction.x, p_direction.y);
    if (norm < 1e-9)
    {
        throw std::invalid_argument("Conveyor '" + name() + "': the direction needs x or y");
    }
    m_direction = Vector3(p_direction.x / norm, p_direction.y / norm, 0.0);
}

} // namespace robotik
