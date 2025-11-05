/**
 * @file Frame.cpp
 * @brief Frame class - Representation of a coordinate frame.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#include "Robotik/Core/Robot/Blueprint/Frame.hpp"
#include "Robotik/Core/Robot/Blueprint/NodeVisitor.hpp"

namespace robotik
{

// ----------------------------------------------------------------------------
void Frame::accept(NodeVisitor& visitor)
{
    visitor.visit(*this);
}

// ----------------------------------------------------------------------------
void Frame::accept(ConstNodeVisitor& visitor) const
{
    visitor.visit(*this);
}

} // namespace robotik
