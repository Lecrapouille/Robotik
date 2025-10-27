/**
 * @file Exception.hpp
 * @brief Exception classes for the Robotik library.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#pragma once

#include <stdexcept>
#include <string>

namespace robotik
{

// ****************************************************************************
//! \brief Base exception class for all Robotik library exceptions.
// ****************************************************************************
class RobotikException: public std::runtime_error
{
public:

    explicit RobotikException(const std::string& message)
        : std::runtime_error("Robotik: " + message)
    {
    }
};

} // namespace robotik