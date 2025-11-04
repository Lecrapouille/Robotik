/**
 * @file RobotExporterFactory.hpp
 * @brief Factory for creating robot exporters based on file extension.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#pragma once

#include "Robotik/Core/Exporters/RobotExporter.hpp"

#include <functional>
#include <memory>
#include <string>
#include <unordered_map>

namespace robotik
{

// ****************************************************************************
//! \brief Factory for creating appropriate robot exporters based on file type.
//!
//! This factory automatically detects the file format from the extension
//! and returns the appropriate exporter. It supports registration of custom
//! exporters for extensibility.
//!
//! Example usage:
//! \code
//!   auto exporter = RobotExporterFactory::create("robot.dot");
//!   exporter->exportTo(robot, "robot.dot");
//! \endcode
// ****************************************************************************
class RobotExporterFactory
{
public:

    // ------------------------------------------------------------------------
    //! \brief Type alias for exporter creator functions.
    // ------------------------------------------------------------------------
    using ExporterCreator = std::function<std::unique_ptr<RobotExporter>()>;

    // ------------------------------------------------------------------------
    //! \brief Create an exporter appropriate for the given filename.
    //! \param p_filename Path to the output file (extension is used to detect
    //! format).
    //! \return A unique pointer to the appropriate exporter, or nullptr if the
    //! format is not supported.
    // ------------------------------------------------------------------------
    static std::unique_ptr<RobotExporter> create(std::string const& p_filename);

    // ------------------------------------------------------------------------
    //! \brief Register a custom exporter for a specific file extension.
    //! \param p_extension File extension (e.g., ".dot", ".urdf").
    //! \param p_creator Function that creates an exporter instance (moved).
    //!
    //! This allows users to add support for custom robot export formats.
    // ------------------------------------------------------------------------
    static void registerExporter(std::string const& p_extension,
                                 ExporterCreator&& p_creator);

private:

    // ------------------------------------------------------------------------
    //! \brief Get the registry of exporters.
    //! \return Reference to the static registry map.
    // ------------------------------------------------------------------------
    static std::unordered_map<std::string, ExporterCreator>& getRegistry();

    // ------------------------------------------------------------------------
    //! \brief Extract file extension from filename.
    //! \param p_filename Filename to extract extension from.
    //! \return Extension in lowercase (e.g., ".dot").
    // ------------------------------------------------------------------------
    static std::string getExtension(std::string const& p_filename);
};

} // namespace robotik
