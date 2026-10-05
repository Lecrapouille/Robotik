// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#pragma once

#include <cmath>
#include <numbers>

// Closed stadium-shaped line on the floor: two straights joined by half
// circles, run counter-clockwise from the start of the lower straight.
struct Track
{
    double straight = 2.0;
    double radius = 0.8;
    double width = 0.025;

    struct Point
    {
        double x = 0.0;
        double y = 0.0;
        double heading = 0.0;
        // Arc length from the start, in [0, perimeter).
        double s = 0.0;
    };

    [[nodiscard]] double perimeter() const
    {
        return 2.0 * straight + 2.0 * std::numbers::pi * radius;
    }

    [[nodiscard]] Point at(double p_s) const;
    [[nodiscard]] Point closest(double p_x, double p_y) const;

    // Signed arc length from @p_from to @p_to, the shorter way round.
    [[nodiscard]] double advance(double p_from, double p_to) const
    {
        double const period = perimeter();
        double delta = std::fmod(p_to - p_from, period);
        if (delta > 0.5 * period)
        {
            delta -= period;
        }
        else if (delta < -0.5 * period)
        {
            delta += period;
        }
        return delta;
    }
};
