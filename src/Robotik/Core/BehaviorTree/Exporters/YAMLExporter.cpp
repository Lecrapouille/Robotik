/**
 * @file YAMLExporter.cpp
 * @brief YAML exporter for behavior trees.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#include "Robotik/Core/BehaviorTree/Exporters/YAMLExporter.hpp"
#include "Robotik/Core/BehaviorTree/BehaviorTree.hpp"

#include <fstream>
#include <yaml-cpp/yaml.h>

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
// YAMLExporter implementation
// ****************************************************************************

//-----------------------------------------------------------------------------
bool YAMLExporter::exportTo(const Tree& tree, const std::string& filename)
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
std::string YAMLExporter::toString(const Tree& tree)
{
    m_error.clear();

    if (!tree.hasRoot())
    {
        m_error = "Tree has no root node";
        return {};
    }

    YAML::Node root;
    YAML::Node btNode;
    m_current_yaml = &btNode;

    tree.getRoot().accept(*this);

    root["BehaviorTree"] = btNode;
    return YAML::Dump(root);
}

// ****************************************************************************
// Visitor implementations
// ****************************************************************************

//-----------------------------------------------------------------------------
void YAMLExporter::visitTree(Tree const& node)
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
void YAMLExporter::visitSequence(Sequence const& node)
{
    YAML::Node content;
    if (!node.name.empty() && node.name != "sequence")
    {
        content["name"] = node.name;
    }

    YAML::Node children;
    for (auto const& child : node.getChildren())
    {
        YAML::Node childNode;
        YAML::Node* prevYaml = m_current_yaml;
        m_current_yaml = &childNode;
        child->accept(*this);
        children.push_back(childNode);
        m_current_yaml = prevYaml;
    }
    content["children"] = children;
    (*m_current_yaml)["Sequence"] = content;
}

//-----------------------------------------------------------------------------
void YAMLExporter::visitReactiveSequence(ReactiveSequence const& node)
{
    // ReactiveSequence exports the same as Sequence
    visitSequence(reinterpret_cast<Sequence const&>(node));
}

//-----------------------------------------------------------------------------
void YAMLExporter::visitStatefulSequence(StatefulSequence const& node)
{
    // StatefulSequence exports the same as Sequence
    visitSequence(reinterpret_cast<Sequence const&>(node));
}

//-----------------------------------------------------------------------------
void YAMLExporter::visitSelector(Selector const& node)
{
    YAML::Node content;
    if (!node.name.empty() && node.name != "selector")
    {
        content["name"] = node.name;
    }

    YAML::Node children;
    for (auto const& child : node.getChildren())
    {
        YAML::Node childNode;
        YAML::Node* prevYaml = m_current_yaml;
        m_current_yaml = &childNode;
        child->accept(*this);
        children.push_back(childNode);
        m_current_yaml = prevYaml;
    }
    content["children"] = children;
    (*m_current_yaml)["Selector"] = content;
}

//-----------------------------------------------------------------------------
void YAMLExporter::visitReactiveSelector(ReactiveSelector const& node)
{
    // ReactiveSelector exports the same as Selector
    visitSelector(reinterpret_cast<Selector const&>(node));
}

//-----------------------------------------------------------------------------
void YAMLExporter::visitStatefulSelector(StatefulSelector const& node)
{
    // StatefulSelector exports the same as Selector
    visitSelector(reinterpret_cast<Selector const&>(node));
}

//-----------------------------------------------------------------------------
void YAMLExporter::visitParallel(Parallel const& node)
{
    YAML::Node content;
    if (!node.name.empty() && node.name != "parallel")
    {
        content["name"] = node.name;
    }
    content["success_threshold"] = node.getMinSuccess();
    content["failure_threshold"] = node.getMinFail();

    YAML::Node children;
    for (auto const& child : node.getChildren())
    {
        YAML::Node childNode;
        YAML::Node* prevYaml = m_current_yaml;
        m_current_yaml = &childNode;
        child->accept(*this);
        children.push_back(childNode);
        m_current_yaml = prevYaml;
    }
    content["children"] = children;
    (*m_current_yaml)["Parallel"] = content;
}

//-----------------------------------------------------------------------------
void YAMLExporter::visitParallelAll(ParallelAll const& node)
{
    YAML::Node content;
    if (!node.name.empty() && node.name != "parallel")
    {
        content["name"] = node.name;
    }
    content["success_on_all"] = node.getSuccessOnAll();
    content["fail_on_all"] = node.getFailOnAll();

    YAML::Node children;
    for (auto const& child : node.getChildren())
    {
        YAML::Node childNode;
        YAML::Node* prevYaml = m_current_yaml;
        m_current_yaml = &childNode;
        child->accept(*this);
        children.push_back(childNode);
        m_current_yaml = prevYaml;
    }
    content["children"] = children;
    (*m_current_yaml)["Parallel"] = content;
}

// ****************************************************************************
// Decorator nodes
// ****************************************************************************

//-----------------------------------------------------------------------------
void YAMLExporter::visitInverter(Inverter const& node)
{
    YAML::Node content;
    if (!node.name.empty() && node.name != "inverter")
    {
        content["name"] = node.name;
    }
    if (node.hasChild())
    {
        YAML::Node child_seq;
        YAML::Node childNode;
        YAML::Node* prevYaml = m_current_yaml;
        m_current_yaml = &childNode;
        node.getChild().accept(*this);
        child_seq.push_back(childNode);
        content["child"] = child_seq;
        m_current_yaml = prevYaml;
    }
    (*m_current_yaml)["Inverter"] = content;
}

//-----------------------------------------------------------------------------
void YAMLExporter::visitRetry(Retry const& node)
{
    YAML::Node content;
    if (!node.name.empty() && node.name != "retry")
    {
        content["name"] = node.name;
    }
    content["attempts"] = node.getAttempts();
    if (node.hasChild())
    {
        YAML::Node child_seq;
        YAML::Node childNode;
        YAML::Node* prevYaml = m_current_yaml;
        m_current_yaml = &childNode;
        node.getChild().accept(*this);
        child_seq.push_back(childNode);
        content["child"] = child_seq;
        m_current_yaml = prevYaml;
    }
    (*m_current_yaml)["Retry"] = content;
}

//-----------------------------------------------------------------------------
void YAMLExporter::visitRepeat(Repeat const& node)
{
    YAML::Node content;
    if (!node.name.empty() && node.name != "repeat")
    {
        content["name"] = node.name;
    }
    content["times"] = node.getRepetitions();
    if (node.hasChild())
    {
        YAML::Node child_seq;
        YAML::Node childNode;
        YAML::Node* prevYaml = m_current_yaml;
        m_current_yaml = &childNode;
        node.getChild().accept(*this);
        child_seq.push_back(childNode);
        content["child"] = child_seq;
        m_current_yaml = prevYaml;
    }
    (*m_current_yaml)["Repeat"] = content;
}

//-----------------------------------------------------------------------------
void YAMLExporter::visitUntilSuccess(UntilSuccess const& node)
{
    YAML::Node content;
    if (!node.name.empty() && node.name != "repeat_until_success")
    {
        content["name"] = node.name;
    }
    if (node.hasChild())
    {
        YAML::Node child_seq;
        YAML::Node childNode;
        YAML::Node* prevYaml = m_current_yaml;
        m_current_yaml = &childNode;
        node.getChild().accept(*this);
        child_seq.push_back(childNode);
        content["child"] = child_seq;
        m_current_yaml = prevYaml;
    }
    (*m_current_yaml)["RepeatUntilSuccess"] = content;
}

//-----------------------------------------------------------------------------
void YAMLExporter::visitUntilFailure(UntilFailure const& node)
{
    YAML::Node content;
    if (!node.name.empty() && node.name != "repeat_until_failure")
    {
        content["name"] = node.name;
    }
    if (node.hasChild())
    {
        YAML::Node child_seq;
        YAML::Node childNode;
        YAML::Node* prevYaml = m_current_yaml;
        m_current_yaml = &childNode;
        node.getChild().accept(*this);
        child_seq.push_back(childNode);
        content["child"] = child_seq;
        m_current_yaml = prevYaml;
    }
    (*m_current_yaml)["RepeatUntilFailure"] = content;
}

//-----------------------------------------------------------------------------
void YAMLExporter::visitForceSuccess(ForceSuccess const& node)
{
    YAML::Node content;
    if (!node.name.empty())
    {
        content["name"] = node.name;
    }
    if (node.hasChild())
    {
        YAML::Node child_seq;
        YAML::Node childNode;
        YAML::Node* prevYaml = m_current_yaml;
        m_current_yaml = &childNode;
        node.getChild().accept(*this);
        child_seq.push_back(childNode);
        content["child"] = child_seq;
        m_current_yaml = prevYaml;
    }
    (*m_current_yaml)["ForceSuccess"] = content;
}

//-----------------------------------------------------------------------------
void YAMLExporter::visitForceFailure(ForceFailure const& node)
{
    YAML::Node content;
    if (!node.name.empty())
    {
        content["name"] = node.name;
    }
    if (node.hasChild())
    {
        YAML::Node child_seq;
        YAML::Node childNode;
        YAML::Node* prevYaml = m_current_yaml;
        m_current_yaml = &childNode;
        node.getChild().accept(*this);
        child_seq.push_back(childNode);
        content["child"] = child_seq;
        m_current_yaml = prevYaml;
    }
    (*m_current_yaml)["ForceFailure"] = content;
}

// ****************************************************************************
// Leaf nodes
// ****************************************************************************

//-----------------------------------------------------------------------------
void YAMLExporter::visitSuccess(Success const& node)
{
    YAML::Node content;
    content["name"] = node.name;
    (*m_current_yaml)["Success"] = content;
}

//-----------------------------------------------------------------------------
void YAMLExporter::visitFailure(Failure const& node)
{
    YAML::Node content;
    content["name"] = node.name;
    (*m_current_yaml)["Failure"] = content;
}

//-----------------------------------------------------------------------------
void YAMLExporter::visitCondition(Condition const& node)
{
    YAML::Node content;
    content["name"] = node.name;
    (*m_current_yaml)["Condition"] = content;
}

//-----------------------------------------------------------------------------
void YAMLExporter::visitSugarAction(SugarAction const& node)
{
    YAML::Node content;
    content["name"] = node.name;
    (*m_current_yaml)["Action"] = content;
}

} // namespace bt
