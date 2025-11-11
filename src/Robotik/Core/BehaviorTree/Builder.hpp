/**
 * @file Builder.hpp
 * @brief Builder class for creating behavior trees from YAML.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#pragma once

#include "Robotik/Core/BehaviorTree/BehaviorTree.hpp"
#include "Robotik/Core/Common/Return.hpp"

namespace YAML
{
class Node;
}

namespace bt
{

// ****************************************************************************
//! \brief Builder class for creating behavior trees from YAML.
// ****************************************************************************
class Builder
{
public:

    // --------------------------------------------------------------------------
    //! \brief Create a behavior tree from a YAML file.
    //! \param[in] factory The factory to create custom nodes.
    //! \param[in] file_path The path to the YAML file.
    //! \param[in] blackboard Optional blackboard to populate from YAML.
    //! \return Return object containing the tree or an error message.
    // --------------------------------------------------------------------------
    static robotik::Return<Tree::Ptr>
    fromFile(NodeFactory const& factory,
             std::string const& file_path,
             Blackboard::Ptr blackboard = nullptr);

    // --------------------------------------------------------------------------
    //! \brief Create a behavior tree from YAML text.
    //! \param[in] factory The factory to create custom nodes.
    //! \param[in] yaml_text The YAML text describing the tree.
    //! \param[in] blackboard Optional blackboard to populate from YAML.
    //! \return Return object containing the tree or an error message.
    // --------------------------------------------------------------------------
    static robotik::Return<Tree::Ptr>
    fromText(NodeFactory const& factory,
             std::string const& yaml_text,
             Blackboard::Ptr blackboard = nullptr);

    // --------------------------------------------------------------------------
    //! \brief Parse a YAML node into a behavior tree node.
    //! \param[in] factory The factory to create custom nodes.
    //! \param[in] node The YAML node to parse.
    //! \return Return object containing the node or an error message.
    // --------------------------------------------------------------------------
    static robotik::Return<Node::Ptr> parseYAMLNode(NodeFactory const& factory,
                                                    YAML::Node const& node);

private:

    // --------------------------------------------------------------------------
    //! \brief Internal parseYAMLNode with blackboard support.
    //! \param[in] factory The factory to create custom nodes.
    //! \param[in] node The YAML node to parse.
    //! \param[in] blackboard Optional blackboard for parameter resolution.
    //! \return Return object containing the node or an error message.
    // --------------------------------------------------------------------------
    static robotik::Return<Node::Ptr> parseYAMLNode(NodeFactory const& factory,
                                                    YAML::Node const& node,
                                                    Blackboard::Ptr blackboard);
};

} // namespace bt