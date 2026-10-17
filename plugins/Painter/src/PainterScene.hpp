// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

//! @file PainterScene.hpp
//! @brief Conventions of the painter demonstration.
//!
//! The world (worlds/painting_lab.yaml) builds the easel, the canvas and the
//! pots; the strokes are trajectories of the scenario, in the frame of the
//! easel. What is left here are the entity ids, the brush orientation over a
//! pot and the paint colours. Frames are URDF Z-up, the world frame.
#pragma once

#include "Robotik/Math/Geometry.hpp"

#include <cmath>
#include <set>
#include <string>
#include <string_view>

namespace painter
{

//! World entity ids.
inline constexpr char const* kCanvas = "canvas";
inline constexpr char const* kEasel = "easel";

//! Tool straight down, for dipping into a pot.
inline robotik::Quaternion brushDown()
{
    return { 0.0, 0.0, 1.0, 0.0 };
}

//! @brief Shared with the mission so a stroke can tint the canvas texture.
//! @c brush and @c painted are the memory of the robot (it cannot see the
//! paint): set by the gestures that succeed, read by the planner.
struct Stroke
{
    std::string color = "red";
    bool painting = false;
    //!< Colour on the brush, empty when clean.
    std::string brush;
    //!< Colours already laid on the canvas.
    std::set<std::string, std::less<>> painted;
};

inline int colorIndex(std::string_view p_name)
{
    if (p_name == "red")
    {
        return 0;
    }
    if (p_name == "black")
    {
        return 1;
    }
    if (p_name == "blue")
    {
        return 2;
    }
    return 3;
}

inline void colorRgb(std::string_view p_name, float& p_r, float& p_g, float& p_b)
{
    switch (colorIndex(p_name))
    {
        case 0:
            p_r = 0.82f;
            p_g = 0.18f;
            p_b = 0.18f;
            break;
        case 1:
            p_r = 0.08f;
            p_g = 0.08f;
            p_b = 0.10f;
            break;
        case 2:
            p_r = 0.10f;
            p_g = 0.22f;
            p_b = 0.62f;
            break;
        default:
            p_r = 0.86f;
            p_g = 0.67f;
            p_b = 0.16f;
            break;
    }
}

} // namespace painter
