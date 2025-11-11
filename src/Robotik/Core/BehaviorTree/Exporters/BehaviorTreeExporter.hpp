/**
 * @file BehaviorTreeExporter.hpp
 * @brief Base interface for behavior tree exporters.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#pragma once

#include <string>

namespace bt
{

// Forward declaration
class Tree;

// ****************************************************************************
//! \brief Base interface for all behavior tree exporters.
//!
//! This interface allows polymorphic handling of different export formats
//! (YAML, BT.CPP XML, Mermaid, etc.). Exporters are responsible for exporting
//! Tree instances to various file formats.
//!
//! Example usage:
//! \code
//!   std::unique_ptr<BehaviorTreeExporter> exporter =
//!   std::make_unique<YAMLExporter>(); if (!exporter->exportTo(tree,
//!   "tree.yaml")) {
//!       std::cerr << "Error: " << exporter->error() << std::endl;
//!   }
//! \endcode
// ****************************************************************************
class BehaviorTreeExporter
{
public:

    // ------------------------------------------------------------------------
    //! \brief Virtual destructor.
    // ------------------------------------------------------------------------
    virtual ~BehaviorTreeExporter() = default;

    // ------------------------------------------------------------------------
    //! \brief Export a behavior tree to a file.
    //! \param tree The tree to export.
    //! \param filename Path to the output file.
    //! \return True if the export was successful, false otherwise.
    //!
    //! Example:
    //! \code
    //!   YAMLExporter exporter;
    //!   if (!exporter.exportTo(tree, "tree.yaml")) {
    //!       std::cerr << exporter.error() << std::endl;
    //!   }
    //! \endcode
    // ------------------------------------------------------------------------
    virtual bool exportTo(const Tree& tree, const std::string& filename) = 0;

    // ------------------------------------------------------------------------
    //! \brief Export a behavior tree to a string.
    //! \param tree The tree to export.
    //! \return The exported tree as a string, or empty string if error.
    //!
    //! Example:
    //! \code
    //!   YAMLExporter exporter;
    //!   std::string yaml = exporter.toString(tree);
    //!   if (yaml.empty()) {
    //!       std::cerr << exporter.error() << std::endl;
    //!   }
    //! \endcode
    // ------------------------------------------------------------------------
    virtual std::string toString(const Tree& tree) = 0;

    // ------------------------------------------------------------------------
    //! \brief Get the error message if exportTo() or toString() failed.
    //! \return Error message string, or empty string if no error occurred.
    // ------------------------------------------------------------------------
    virtual std::string error() const = 0;
};

} // namespace bt
