/**
 * @file Builder.hpp
 * @brief Builder class for creating behavior trees from YAML.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#pragma once

#include "Robotik/Core/BehaviorTree/Builder/Factory.hpp"
#include "Robotik/Core/Common/Return.hpp"
#include "Robotik/Core/BehaviorTree/Core/Tree.hpp"

namespace YAML {
class Node;
}

namespace robotik::bt {

struct SubTreeRegistry;

// ****************************************************************************
//! \brief Builder class for creating behavior trees from YAML.
//!
//! The builder is a utility class that is used to create behavior trees from
//! YAML file or a YAML text.
//!
//! Key features:
//! - Create a behavior tree from a YAML file or a YAML text.
//! - Parse a YAML node into a behavior tree node.
//! - Support for builtin nodes.
//! - Support for custom nodes (it must access to your classes to create them).
//! - Support for blackboard.
//! - Support for subtrees.
// ****************************************************************************
class Builder
{
public:

    // --------------------------------------------------------------------------
    //! \brief Create a behavior tree from a YAML file.
    //! \param[in] p_factory The factory to create custom nodes.
    //! \param[in] p_file_path The path to the YAML file.
    //! \param[in] p_blackboard Optional blackboard to populate from YAML.
    //! \return Return object containing the tree or an error message.
    // --------------------------------------------------------------------------
    static robotik::Return<Tree::Ptr>
    fromFile(NodeFactory const& p_factory,
             std::string const& p_file_path,
             Blackboard::Ptr p_blackboard = nullptr);

    // --------------------------------------------------------------------------
    //! \brief Create a behavior tree from YAML text.
    //! \param[in] p_factory The factory to create custom nodes.
    //! \param[in] p_yaml_text The YAML text describing the tree.
    //! \param[in] p_blackboard Optional blackboard to populate from YAML.
    //! \return Return object containing the tree or an error message.
    // --------------------------------------------------------------------------
    static robotik::Return<Tree::Ptr>
    fromText(NodeFactory const& p_factory,
             std::string const& p_yaml_text,
             Blackboard::Ptr p_blackboard = nullptr);

    // --------------------------------------------------------------------------
    //! \brief Parse a YAML node into a behavior tree node.
    //! \param[in] p_factory The factory to create custom nodes.
    //! \param[in] p_node The YAML node to parse.
    //! \return Return object containing the node or an error message.
    // --------------------------------------------------------------------------
    static robotik::Return<Node::Ptr>
    parseYAMLNode(NodeFactory const& p_factory, YAML::Node const& p_node);

private:

    // --------------------------------------------------------------------------
    //! \brief Internal parseYAMLNode with blackboard support.
    //! \param[in] p_factory The factory to create custom nodes.
    //! \param[in] p_node The YAML node to parse.
    //! \param[in] p_blackboard Optional blackboard for parameter resolution.
    //! \return Return object containing the node or an error message.
    // --------------------------------------------------------------------------
    static robotik::Return<Node::Ptr>
    parseYAMLNode(NodeFactory const& p_factory,
                  YAML::Node const& p_node,
                  Blackboard::Ptr p_blackboard,
                  SubTreeRegistry const* p_subtrees);
};

} // namespace robotik::bt