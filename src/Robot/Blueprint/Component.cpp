/**
 * @file Component.cpp
 * @brief Implementation of Component classes (Sensor, Actuator).
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#include "Robotik/Robot/Blueprint/Component.hpp"
#include "Robotik/Robot/Blueprint/NodeVisitor.hpp"

namespace robotik
{

// ----------------------------------------------------------------------------
void Sensor::accept(NodeVisitor& visitor)
{
    visitor.visit(*this);
}

// ----------------------------------------------------------------------------
void Sensor::accept(ConstNodeVisitor& visitor) const
{
    visitor.visit(*this);
}

// ----------------------------------------------------------------------------
void Actuator::accept(NodeVisitor& visitor)
{
    visitor.visit(*this);
}

// ----------------------------------------------------------------------------
void Actuator::accept(ConstNodeVisitor& visitor) const
{
    visitor.visit(*this);
}

} // namespace robotik
