/**
 * @file Component.cpp
 * @brief Implementation of Component classes (Sensor, Actuator).
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#include "Robotik/Core/Robot/Blueprint/Component.hpp"
#include "Robotik/Core/Robot/RobotVisitor.hpp"

namespace robotik
{

// ----------------------------------------------------------------------------
void Sensor::accept(RobotVisitor& visitor)
{
    visitor.visit(*this);
}

// ----------------------------------------------------------------------------
void Sensor::accept(ConstRobotVisitor& visitor) const
{
    visitor.visit(*this);
}

// ----------------------------------------------------------------------------
void Actuator::accept(RobotVisitor& visitor)
{
    visitor.visit(*this);
}

// ----------------------------------------------------------------------------
void Actuator::accept(ConstRobotVisitor& visitor) const
{
    visitor.visit(*this);
}

} // namespace robotik
