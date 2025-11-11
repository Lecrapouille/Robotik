/**
 * @file MermaidExporter.cpp
 * @brief Mermaid diagram exporter for behavior trees.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#include "Robotik/Core/BehaviorTree/Exporters/MermaidExporter.hpp"
#include "Robotik/Core/BehaviorTree/BehaviorTree.hpp"

#include <fstream>

namespace bt
{

// ****************************************************************************
// Helper functions
// ****************************************************************************
namespace
{

//-----------------------------------------------------------------------------
bool writeToFile(std::string const& content, std::string const& filename)
{
    std::ofstream file(filename);
    if (!file.is_open())
    {
        return false;
    }
    file << content;
    return true;
}

} // anonymous namespace

// ****************************************************************************
// MermaidExporter implementation
// ****************************************************************************

//-----------------------------------------------------------------------------
bool MermaidExporter::exportTo(const Tree& tree, const std::string& filename)
{
    std::string content = toString(tree);
    if (content.empty())
    {
        return false;
    }

    if (!writeToFile(content, filename))
    {
        m_error = "Failed to write file: " + filename;
        return false;
    }

    return true;
}

//-----------------------------------------------------------------------------
std::string MermaidExporter::toString(const Tree& tree)
{
    m_error.clear();
    m_result.str("");
    m_result.clear();
    m_counter = 0;

    if (!tree.hasRoot())
    {
        m_error = "Tree has no root node";
        return {};
    }

    m_result << "graph TD\n";

    // Define the styles for each type of node with classDef
    m_result << "    classDef sequence "
                "fill:#b3e0ff,stroke:#0066cc,stroke-width:2px,color:#000000,"
                "font-weight:bold\n";
    m_result << "    classDef selector "
                "fill:#ffcccc,stroke:#cc0000,stroke-width:2px,color:#000000,"
                "font-weight:bold\n";
    m_result << "    classDef parallel "
                "fill:#d9b3ff,stroke:#6600cc,stroke-width:2px,color:#000000,"
                "font-weight:bold\n";
    m_result << "    classDef decorator "
                "fill:#ffffb3,stroke:#cccc00,stroke-width:2px,color:#000000,"
                "font-weight:bold\n";
    m_result << "    classDef condition "
                "fill:#b3ffb3,stroke:#00cc00,stroke-width:2px,color:#000000,"
                "font-weight:bold\n";
    m_result << "    classDef action "
                "fill:#ffb3d9,stroke:#cc0066,stroke-width:2px,color:#000000,"
                "font-weight:bold\n";

    tree.getRoot().accept(*this);

    return m_result.str();
}

// ****************************************************************************
// Helper method
// ****************************************************************************

//-----------------------------------------------------------------------------
void MermaidExporter::generateNodeHelper(Node const* node,
                                         size_t parent_id,
                                         std::string const& node_class)
{
    size_t current_id = ++m_counter;
    std::string node_name = node->name;

    // Add the node definition
    m_result << "    node" << current_id << "[\"" << node_name << "\"]\n";

    // Apply the class to the node
    if (!node_class.empty())
    {
        m_result << "    class node" << current_id << " " << node_class << "\n";
    }

    // Add the connection to the parent (if it's not the root)
    if (parent_id > 0)
    {
        m_result << "    node" << parent_id << " --> node" << current_id
                 << "\n";
    }
}

// ****************************************************************************
// Visitor implementations
// ****************************************************************************

//-----------------------------------------------------------------------------
void MermaidExporter::visitTree(Tree const& node)
{
    if (node.hasRoot())
    {
        m_parent_id = 0;
        node.getRoot().accept(*this);
    }
}

// ****************************************************************************
// Composite nodes
// ****************************************************************************

//-----------------------------------------------------------------------------
void MermaidExporter::visitSequence(Sequence const& node)
{
    size_t parent_id = m_parent_id;
    generateNodeHelper(&node, parent_id, "sequence");
    size_t current_id = m_counter;

    // Process children
    for (auto const& child : node.getChildren())
    {
        m_parent_id = current_id;
        child->accept(*this);
    }
    m_parent_id = parent_id;
}

//-----------------------------------------------------------------------------
void MermaidExporter::visitReactiveSequence(ReactiveSequence const& node)
{
    visitSequence(reinterpret_cast<Sequence const&>(node));
}

//-----------------------------------------------------------------------------
void MermaidExporter::visitStatefulSequence(StatefulSequence const& node)
{
    visitSequence(reinterpret_cast<Sequence const&>(node));
}

//-----------------------------------------------------------------------------
void MermaidExporter::visitSelector(Selector const& node)
{
    size_t parent_id = m_parent_id;
    generateNodeHelper(&node, parent_id, "selector");
    size_t current_id = m_counter;

    // Process children
    for (auto const& child : node.getChildren())
    {
        m_parent_id = current_id;
        child->accept(*this);
    }
    m_parent_id = parent_id;
}

//-----------------------------------------------------------------------------
void MermaidExporter::visitReactiveSelector(ReactiveSelector const& node)
{
    visitSelector(reinterpret_cast<Selector const&>(node));
}

//-----------------------------------------------------------------------------
void MermaidExporter::visitStatefulSelector(StatefulSelector const& node)
{
    visitSelector(reinterpret_cast<Selector const&>(node));
}

//-----------------------------------------------------------------------------
void MermaidExporter::visitParallel(Parallel const& node)
{
    size_t parent_id = m_parent_id;
    generateNodeHelper(&node, parent_id, "parallel");
    size_t current_id = m_counter;

    // Process children
    for (auto const& child : node.getChildren())
    {
        m_parent_id = current_id;
        child->accept(*this);
    }
    m_parent_id = parent_id;
}

//-----------------------------------------------------------------------------
void MermaidExporter::visitParallelAll(ParallelAll const& node)
{
    size_t parent_id = m_parent_id;
    generateNodeHelper(&node, parent_id, "parallel");
    size_t current_id = m_counter;

    // Process children
    for (auto const& child : node.getChildren())
    {
        m_parent_id = current_id;
        child->accept(*this);
    }
    m_parent_id = parent_id;
}

// ****************************************************************************
// Decorator nodes
// ****************************************************************************

//-----------------------------------------------------------------------------
void MermaidExporter::visitInverter(Inverter const& node)
{
    size_t parent_id = m_parent_id;
    generateNodeHelper(&node, parent_id, "decorator");
    size_t current_id = m_counter;

    if (node.hasChild())
    {
        m_parent_id = current_id;
        node.getChild().accept(*this);
    }
    m_parent_id = parent_id;
}

//-----------------------------------------------------------------------------
void MermaidExporter::visitRetry(Retry const& node)
{
    size_t parent_id = m_parent_id;
    generateNodeHelper(&node, parent_id, "decorator");
    size_t current_id = m_counter;

    if (node.hasChild())
    {
        m_parent_id = current_id;
        node.getChild().accept(*this);
    }
    m_parent_id = parent_id;
}

//-----------------------------------------------------------------------------
void MermaidExporter::visitRepeat(Repeat const& node)
{
    size_t parent_id = m_parent_id;
    generateNodeHelper(&node, parent_id, "decorator");
    size_t current_id = m_counter;

    if (node.hasChild())
    {
        m_parent_id = current_id;
        node.getChild().accept(*this);
    }
    m_parent_id = parent_id;
}

//-----------------------------------------------------------------------------
void MermaidExporter::visitUntilSuccess(UntilSuccess const& node)
{
    size_t parent_id = m_parent_id;
    generateNodeHelper(&node, parent_id, "decorator");
    size_t current_id = m_counter;

    if (node.hasChild())
    {
        m_parent_id = current_id;
        node.getChild().accept(*this);
    }
    m_parent_id = parent_id;
}

//-----------------------------------------------------------------------------
void MermaidExporter::visitUntilFailure(UntilFailure const& node)
{
    size_t parent_id = m_parent_id;
    generateNodeHelper(&node, parent_id, "decorator");
    size_t current_id = m_counter;

    if (node.hasChild())
    {
        m_parent_id = current_id;
        node.getChild().accept(*this);
    }
    m_parent_id = parent_id;
}

//-----------------------------------------------------------------------------
void MermaidExporter::visitForceSuccess(ForceSuccess const& node)
{
    size_t parent_id = m_parent_id;
    generateNodeHelper(&node, parent_id, "decorator");
    size_t current_id = m_counter;

    if (node.hasChild())
    {
        m_parent_id = current_id;
        node.getChild().accept(*this);
    }
    m_parent_id = parent_id;
}

//-----------------------------------------------------------------------------
void MermaidExporter::visitForceFailure(ForceFailure const& node)
{
    size_t parent_id = m_parent_id;
    generateNodeHelper(&node, parent_id, "decorator");
    size_t current_id = m_counter;

    if (node.hasChild())
    {
        m_parent_id = current_id;
        node.getChild().accept(*this);
    }
    m_parent_id = parent_id;
}

// ****************************************************************************
// Leaf nodes
// ****************************************************************************

//-----------------------------------------------------------------------------
void MermaidExporter::visitSuccess(Success const& node)
{
    generateNodeHelper(&node, m_parent_id, "action");
}

//-----------------------------------------------------------------------------
void MermaidExporter::visitFailure(Failure const& node)
{
    generateNodeHelper(&node, m_parent_id, "action");
}

//-----------------------------------------------------------------------------
void MermaidExporter::visitCondition(Condition const& node)
{
    generateNodeHelper(&node, m_parent_id, "condition");
}

} // namespace bt
