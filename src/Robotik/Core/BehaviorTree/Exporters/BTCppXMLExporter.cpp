/**
 * @file BTCppXMLExporter.cpp
 * @brief BehaviorTree.CPP XML exporter for behavior trees.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#include "Robotik/Core/BehaviorTree/Exporters/BTCppXMLExporter.hpp"
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
// BTCppXMLExporter implementation
// ****************************************************************************

//-----------------------------------------------------------------------------
bool BTCppXMLExporter::exportTo(const Tree& tree, const std::string& filename)
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
std::string BTCppXMLExporter::toString(const Tree& tree)
{
    m_error.clear();
    m_xml.str("");
    m_xml.clear();
    m_indent = 4;

    if (!tree.hasRoot())
    {
        m_error = "Tree has no root node";
        return {};
    }

    m_xml << "<?xml version=\"1.0\" ?>" << std::endl;
    m_xml << "<root main_tree_to_execute=\"MainTree\">" << std::endl;
    m_xml << "  <BehaviorTree ID=\"MainTree\">" << std::endl;

    tree.getRoot().accept(*this);

    m_xml << "  </BehaviorTree>" << std::endl;
    m_xml << "</root>" << std::endl;

    return m_xml.str();
}

// ****************************************************************************
// Helper method
// ****************************************************************************

//-----------------------------------------------------------------------------
std::string BTCppXMLExporter::spaces() const
{
    return std::string(static_cast<size_t>(m_indent), ' ');
}

// ****************************************************************************
// Visitor implementations
// ****************************************************************************

//-----------------------------------------------------------------------------
void BTCppXMLExporter::visitTree(Tree const& node)
{
    if (node.hasRoot())
    {
        node.getRoot().accept(*this);
    }
}

// ****************************************************************************
// Composite nodes
// ****************************************************************************

//-----------------------------------------------------------------------------
void BTCppXMLExporter::visitSequence(Sequence const& node)
{
    m_xml << spaces() << "<Sequence>" << std::endl;
    m_indent += 2;
    for (auto const& child : node.getChildren())
    {
        child->accept(*this);
    }
    m_indent -= 2;
    m_xml << spaces() << "</Sequence>" << std::endl;
}

//-----------------------------------------------------------------------------
void BTCppXMLExporter::visitReactiveSequence(ReactiveSequence const& node)
{
    visitSequence(reinterpret_cast<Sequence const&>(node));
}

//-----------------------------------------------------------------------------
void BTCppXMLExporter::visitStatefulSequence(StatefulSequence const& node)
{
    visitSequence(reinterpret_cast<Sequence const&>(node));
}

//-----------------------------------------------------------------------------
void BTCppXMLExporter::visitSelector(Selector const& node)
{
    // BT.CPP uses Fallback instead of Selector
    m_xml << spaces() << "<Fallback>" << std::endl;
    m_indent += 2;
    for (auto const& child : node.getChildren())
    {
        child->accept(*this);
    }
    m_indent -= 2;
    m_xml << spaces() << "</Fallback>" << std::endl;
}

//-----------------------------------------------------------------------------
void BTCppXMLExporter::visitReactiveSelector(ReactiveSelector const& node)
{
    visitSelector(reinterpret_cast<Selector const&>(node));
}

//-----------------------------------------------------------------------------
void BTCppXMLExporter::visitStatefulSelector(StatefulSelector const& node)
{
    visitSelector(reinterpret_cast<Selector const&>(node));
}

//-----------------------------------------------------------------------------
void BTCppXMLExporter::visitParallel(Parallel const& node)
{
    m_xml << spaces() << "<Parallel success_threshold=\""
          << node.getMinSuccess() << "\" failure_threshold=\""
          << node.getMinFail() << "\">" << std::endl;
    m_indent += 2;
    for (auto const& child : node.getChildren())
    {
        child->accept(*this);
    }
    m_indent -= 2;
    m_xml << spaces() << "</Parallel>" << std::endl;
}

//-----------------------------------------------------------------------------
void BTCppXMLExporter::visitParallelAll(ParallelAll const& node)
{
    m_xml << spaces() << "<Parallel success_threshold=\""
          << (node.getSuccessOnAll() ? node.getChildren().size() : 1)
          << "\" failure_threshold=\""
          << (node.getFailOnAll() ? node.getChildren().size() : 1) << "\">"
          << std::endl;
    m_indent += 2;
    for (auto const& child : node.getChildren())
    {
        child->accept(*this);
    }
    m_indent -= 2;
    m_xml << spaces() << "</Parallel>" << std::endl;
}

// ****************************************************************************
// Decorator nodes
// ****************************************************************************

//-----------------------------------------------------------------------------
void BTCppXMLExporter::visitInverter(Inverter const& node)
{
    m_xml << spaces() << "<Inverter>" << std::endl;
    m_indent += 2;
    if (node.hasChild())
    {
        node.getChild().accept(*this);
    }
    m_indent -= 2;
    m_xml << spaces() << "</Inverter>" << std::endl;
}

//-----------------------------------------------------------------------------
void BTCppXMLExporter::visitRetry(Retry const& node)
{
    m_xml << spaces() << "<RetryUntilSuccessful num_attempts=\""
          << node.getAttempts() << "\">" << std::endl;
    m_indent += 2;
    if (node.hasChild())
    {
        node.getChild().accept(*this);
    }
    m_indent -= 2;
    m_xml << spaces() << "</RetryUntilSuccessful>" << std::endl;
}

//-----------------------------------------------------------------------------
void BTCppXMLExporter::visitRepeat(Repeat const& node)
{
    m_xml << spaces() << "<Repeat num_cycles=\"" << node.getRepetitions()
          << "\">" << std::endl;
    m_indent += 2;
    if (node.hasChild())
    {
        node.getChild().accept(*this);
    }
    m_indent -= 2;
    m_xml << spaces() << "</Repeat>" << std::endl;
}

//-----------------------------------------------------------------------------
void BTCppXMLExporter::visitUntilSuccess(UntilSuccess const& node)
{
    m_xml << spaces() << "<RepeatUntilSuccess>" << std::endl;
    m_indent += 2;
    if (node.hasChild())
    {
        node.getChild().accept(*this);
    }
    m_indent -= 2;
    m_xml << spaces() << "</RepeatUntilSuccess>" << std::endl;
}

//-----------------------------------------------------------------------------
void BTCppXMLExporter::visitUntilFailure(UntilFailure const& node)
{
    m_xml << spaces() << "<RepeatUntilFailure>" << std::endl;
    m_indent += 2;
    if (node.hasChild())
    {
        node.getChild().accept(*this);
    }
    m_indent -= 2;
    m_xml << spaces() << "</RepeatUntilFailure>" << std::endl;
}

//-----------------------------------------------------------------------------
void BTCppXMLExporter::visitForceSuccess(ForceSuccess const& node)
{
    m_xml << spaces() << "<ForceSuccess>" << std::endl;
    m_indent += 2;
    if (node.hasChild())
    {
        node.getChild().accept(*this);
    }
    m_indent -= 2;
    m_xml << spaces() << "</ForceSuccess>" << std::endl;
}

//-----------------------------------------------------------------------------
void BTCppXMLExporter::visitForceFailure(ForceFailure const& node)
{
    m_xml << spaces() << "<ForceFailure>" << std::endl;
    m_indent += 2;
    if (node.hasChild())
    {
        node.getChild().accept(*this);
    }
    m_indent -= 2;
    m_xml << spaces() << "</ForceFailure>" << std::endl;
}

// ****************************************************************************
// Leaf nodes
// ****************************************************************************

//-----------------------------------------------------------------------------
void BTCppXMLExporter::visitSuccess(Success const& node)
{
    m_xml << spaces() << "<AlwaysSuccess/>" << std::endl;
}

//-----------------------------------------------------------------------------
void BTCppXMLExporter::visitFailure(Failure const& node)
{
    m_xml << spaces() << "<AlwaysFailure/>" << std::endl;
}

//-----------------------------------------------------------------------------
void BTCppXMLExporter::visitCondition(Condition const& node)
{
    m_xml << spaces() << "<Condition ID=\"" << node.name << "\"/>" << std::endl;
}

} // namespace bt
