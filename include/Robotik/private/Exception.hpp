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