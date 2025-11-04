/**
 * @file RobotExporter.hpp
 * @brief Base interface for robot exporters.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#pragma once

#include "Robotik/Core/Robot/Robot.hpp"

#include <string>

namespace robotik
{

// ****************************************************************************
//! \brief Base interface for all robot exporters.
//!
//! This interface allows polymorphic handling of different robot export formats
//! (DOT, URDF, SDF, etc.). Exporters are responsible for exporting Robot
//! instances to various file formats.
//!
//! Example usage:
//! \code
//!   std::unique_ptr<RobotExporter> exporter = std::make_unique<DotExporter>();
//!   if (!exporter->exportTo(robot, "robot.dot")) {
//!       std::cerr << "Error: " << exporter->error() << std::endl;
//!   }
//! \endcode
// ****************************************************************************
class RobotExporter
{
public:

    // ------------------------------------------------------------------------
    //! \brief Virtual destructor.
    // ------------------------------------------------------------------------
    virtual ~RobotExporter() = default;

    // ------------------------------------------------------------------------
    //! \brief Export a robot to a file.
    //! \param robot The robot to export (can be a derived Robot type).
    //! \param filename Path to the output file.
    //! \return True if the export was successful, false otherwise.
    //!
    //! This method supports polymorphic export of Robot and its derived
    //! classes. The exporter accesses the robot's blueprint through the
    //! robot.blueprint() method.
    //!
    //! Example:
    //! \code
    //!   Robot robot("my_robot", std::move(blueprint));
    //!   DotExporter exporter;
    //!   if (!exporter.exportTo(robot, "robot.dot")) {
    //!       std::cerr << exporter.error() << std::endl;
    //!   }
    //! \endcode
    // ------------------------------------------------------------------------
    virtual bool exportTo(const Robot& robot, const std::string& filename) = 0;

    // ------------------------------------------------------------------------
    //! \brief Get the error message if exportTo() failed.
    //! \return Error message string, or empty string if no error occurred.
    // ------------------------------------------------------------------------
    virtual std::string error() const = 0;
};

} // namespace robotik
