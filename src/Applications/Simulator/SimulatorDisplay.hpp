#pragma once

#include "Compages/Core/Vector.hpp"

#define VIEW_CLEAR_R 0.12f
#define VIEW_CLEAR_G 0.14f
#define VIEW_CLEAR_B 0.18f
#define VIEW_CLEAR_A 1.0f

inline compages::core::Vector4f viewClearColor()
{
    return { VIEW_CLEAR_R, VIEW_CLEAR_G, VIEW_CLEAR_B, VIEW_CLEAR_A };
}
