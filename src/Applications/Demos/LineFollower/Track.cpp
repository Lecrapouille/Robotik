// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "Track.hpp"

#include <algorithm>

Track::Point Track::at(double p_s) const
{
    double const period = perimeter();
    double s = std::fmod(p_s, period);
    if (s < 0.0)
    {
        s += period;
    }
    double const half = 0.5 * straight;
    double const arc = std::numbers::pi * radius;
    double const pi = std::numbers::pi;

    if (s < straight)
    {
        return { -half + s, -radius, 0.0, s };
    }
    double u = s - straight;
    if (u < arc)
    {
        double const phi = -0.5 * pi + u / radius;
        return { half + radius * std::cos(phi), radius * std::sin(phi), phi + 0.5 * pi, s };
    }
    u -= arc;
    if (u < straight)
    {
        return { half - u, radius, pi, s };
    }
    u -= straight;
    double const phi = 0.5 * pi + u / radius;
    return { -half + radius * std::cos(phi), radius * std::sin(phi), phi + 0.5 * pi, s };
}

Track::Point Track::closest(double p_x, double p_y) const
{
    double const half = 0.5 * straight;
    double const arc = std::numbers::pi * radius;
    double const pi = std::numbers::pi;

    if (p_x >= half)
    {
        double const phi = std::clamp(std::atan2(p_y, p_x - half), -0.5 * pi, 0.5 * pi);
        return at(straight + (phi + 0.5 * pi) * radius);
    }
    if (p_x <= -half)
    {
        double phi = std::atan2(p_y, p_x + half);
        if (phi < 0.0)
        {
            phi += 2.0 * pi;
        }
        phi = std::clamp(phi, 0.5 * pi, 1.5 * pi);
        return at(2.0 * straight + arc + (phi - 0.5 * pi) * radius);
    }
    if (p_y < 0.0)
    {
        return at(p_x + half);
    }
    return at(straight + arc + (half - p_x));
}
