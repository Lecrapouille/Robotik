/**
 * @file VisualizerClient.cpp
 * @brief Implementation of TCP client for behavior tree visualization.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 */

#include "Robotik/Core/BehaviorTree/Network/VisualizerClient.hpp"
#include "Robotik/Core/BehaviorTree/Builder/Exporter.hpp"
#include "Robotik/Core/BehaviorTree/Core/Composite.hpp"
#include "Robotik/Core/BehaviorTree/Core/Decorator.hpp"
#include "Robotik/Core/BehaviorTree/Core/Tree.hpp"

namespace robotik::bt {

// ----------------------------------------------------------------------------
bool VisualizerClient::sendTree(Tree const& p_tree)
{
    if (!m_connected)
    {
        return false;
    }

    std::string yaml = Exporter::toYAML(p_tree);
    return sendMessage('T', yaml);
}

// ----------------------------------------------------------------------------
void VisualizerClient::sendStateChanges(Tree const& p_tree)
{
    if (!m_connected || !p_tree.hasRoot())
    {
        return;
    }

    std::ostringstream states;
    collectNodeStates(p_tree.getRoot(), states);

    std::string stateStr = states.str();
    if (!stateStr.empty())
    {
        // Remove trailing comma
        stateStr.pop_back();
        sendMessage('S', stateStr);
    }
}

// ----------------------------------------------------------------------------
void VisualizerClient::collectNodeStates(Node const& p_node,
                                         std::ostringstream& p_states)
{
    // Add this node's state: "id:status,"
    p_states << p_node.id() << ":" << static_cast<int>(p_node.status()) << ",";

    // Recurse into children for Composite nodes
    if (auto const* composite = dynamic_cast<Composite const*>(&p_node))
    {
        for (auto const& child : composite->getChildren())
        {
            collectNodeStates(*child, p_states);
        }
    }

    // Recurse into child for Decorator nodes
    if (auto const* decorator = dynamic_cast<Decorator const*>(&p_node))
    {
        if (decorator->hasChild())
        {
            collectNodeStates(decorator->getChild(), p_states);
        }
    }

    // Recurse into SubTree nodes
    if (auto const* subtree = dynamic_cast<SubTreeNode const*>(&p_node))
    {
        if (subtree->handle() && subtree->handle()->tree().hasRoot())
        {
            collectNodeStates(subtree->handle()->tree().getRoot(), p_states);
        }
    }
}

} // namespace robotik::bt
