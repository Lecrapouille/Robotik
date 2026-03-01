/**
 * @file Exporter.hpp
 * @brief Export behavior trees to various formats (YAML, Mermaid).
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 */

#pragma once

#include "Robotik/Core/BehaviorTree/BehaviorTree.hpp"

#include <string>

namespace robotik::bt {

// ****************************************************************************
//! \brief Utility class to export behavior trees to various formats.
//!
//! The Exporter provides static methods to convert behavior trees into:
//! - YAML format (compatible with Builder for round-trip)
//! - Mermaid diagram format (for visualization)
//!
//! Usage:
//! \code
//!   bt::Tree tree;
//!   // ... build tree ...
//!
//!   // Export to YAML string
//!   std::string yaml = bt::Exporter::toYAML(tree);
//!
//!   // Export to file
//!   bt::Exporter::toYAMLFile(tree, "my_tree.yaml");
//!
//!   // Export to Mermaid diagram
//!   std::string mermaid = bt::Exporter::toMermaid(tree);
//! \endcode
// ****************************************************************************
class Exporter
{
public:

    // ------------------------------------------------------------------------
    //! \brief Export a behavior tree to YAML format.
    //! Includes BehaviorTree structure, Blackboard data, and node IDs.
    //! \param[in] p_tree The tree to export.
    //! \return YAML string representation.
    // ------------------------------------------------------------------------
    static std::string toYAML(Tree const& p_tree);

    // ------------------------------------------------------------------------
    //! \brief Export a behavior tree to a YAML file.
    //! \param[in] p_tree The tree to export.
    //! \param[in] p_path The file path to write to.
    //! \return true if successful, false on error.
    // ------------------------------------------------------------------------
    static bool toYAMLFile(Tree const& p_tree, std::string const& p_path);

    // ------------------------------------------------------------------------
    //! \brief Export only the tree structure to YAML (no Blackboard).
    //! \param[in] p_tree The tree to export.
    //! \return YAML string representation of the structure only.
    // ------------------------------------------------------------------------
    static std::string toYAMLStructure(Tree const& p_tree);

    // ------------------------------------------------------------------------
    //! \brief Export a blackboard to YAML format.
    //! \param[in] p_blackboard The blackboard to export.
    //! \return YAML string representation.
    // ------------------------------------------------------------------------
    static std::string blackboardToYAML(Blackboard const& p_blackboard);

    // ------------------------------------------------------------------------
    //! \brief Export a behavior tree to Mermaid diagram format.
    //! \param[in] p_tree The tree to export.
    //! \return Mermaid diagram string.
    // ------------------------------------------------------------------------
    static std::string toMermaid(Tree const& p_tree);
};

} // namespace robotik::bt
