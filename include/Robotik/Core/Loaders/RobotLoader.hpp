/**
 * @file RobotLoader.hpp
 * @brief Base interface for robot file loaders.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#pragma once

#include <memory>
#include <string>

// Forward declaration
namespace robotik
{
class Robot;
}

namespace robotik::loader
{

// ****************************************************************************
//! \brief Base interface for all robot loaders.
//!
//! This interface allows polymorphic handling of different robot file formats
//! (URDF, SDF, MJCF, etc.). Loaders are responsible for parsing robot
//! description files and creating Robot instances.
//!
//! Example usage:
//! \code
//!   std::unique_ptr<RobotLoader> loader = std::make_unique<URDFLoader>();
//!   auto robot = loader->load("robot.urdf");
//!   if (!robot) {
//!       std::cerr << "Error: " << loader->error() << std::endl;
//!   }
//! \endcode
// ****************************************************************************
class RobotLoader
{
public:

    // ------------------------------------------------------------------------
    //! \brief Virtual destructor.
    // ------------------------------------------------------------------------
    virtual ~RobotLoader() = default;

    // ------------------------------------------------------------------------
    //! \brief Load a robot from a file.
    //! \param p_filename Path to the robot description file.
    //! \return A unique pointer to the loaded Robot, or nullptr on failure.
    // ------------------------------------------------------------------------
    virtual std::unique_ptr<robotik::Robot>
    load(const std::string& p_filename) = 0;

    // ------------------------------------------------------------------------
    //! \brief Get the error message if load() failed.
    //! \return Error message string, or empty string if no error occurred.
    // ------------------------------------------------------------------------
    virtual std::string error() const = 0;
};

} // namespace robotik::loader
