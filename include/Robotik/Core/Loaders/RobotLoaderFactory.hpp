/**
 * @file RobotLoaderFactory.hpp
 * @brief Factory for creating robot loaders based on file extension.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#pragma once

#include "Robotik/Core/Loaders/RobotLoader.hpp"

#include <functional>
#include <memory>
#include <string>
#include <unordered_map>

namespace robotik::loader
{

// ****************************************************************************
//! \brief Factory for creating appropriate robot loaders based on file type.
//!
//! This factory automatically detects the file format from the extension
//! and returns the appropriate loader. It supports registration of custom
//! loaders for extensibility.
//!
//! Example usage:
//! \code
//!   auto loader = RobotLoaderFactory::create("robot.urdf");
//!   auto robot = loader->load("robot.urdf");
//! \endcode
// ****************************************************************************
class RobotLoaderFactory
{
public:

    // ------------------------------------------------------------------------
    //! \brief Type alias for loader creator functions.
    // ------------------------------------------------------------------------
    using LoaderCreator = std::function<std::unique_ptr<RobotLoader>()>;

    // ------------------------------------------------------------------------
    //! \brief Create a loader appropriate for the given filename.
    //! \param p_filename Path to the robot file (extension is used to detect
    //! format).
    //! \return A unique pointer to the appropriate loader, or nullptr if the
    //! format is not supported.
    // ------------------------------------------------------------------------
    static std::unique_ptr<RobotLoader> create(const std::string& p_filename);

    // ------------------------------------------------------------------------
    //! \brief Register a custom loader for a specific file extension.
    //! \param p_extension File extension (e.g., ".urdf", ".sdf").
    //! \param p_creator Function that creates a loader instance.
    //!
    //! This allows users to add support for custom robot file formats.
    // ------------------------------------------------------------------------
    static void registerLoader(const std::string& p_extension,
                               LoaderCreator p_creator);

private:

    // ------------------------------------------------------------------------
    //! \brief Get the registry of loaders.
    //! \return Reference to the static registry map.
    // ------------------------------------------------------------------------
    static std::unordered_map<std::string, LoaderCreator>& getRegistry();

    // ------------------------------------------------------------------------
    //! \brief Extract file extension from filename.
    //! \param p_filename Filename to extract extension from.
    //! \return Extension in lowercase (e.g., ".urdf").
    // ------------------------------------------------------------------------
    static std::string getExtension(const std::string& p_filename);
};

} // namespace robotik::loader
