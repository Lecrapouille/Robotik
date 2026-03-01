/**
 * @file Exporter.cpp
 * @brief Implementation of behavior tree export functionality.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 */

#include "Robotik/Core/BehaviorTree/Builder/Exporter.hpp"

#include <fstream>
#include <sstream>

namespace robotik::bt {

// ****************************************************************************
//! \brief Visitor that exports a behavior tree to YAML format.
// ****************************************************************************
class YamlExportVisitor: public ConstBehaviorTreeVisitor
{
public:

    std::stringstream yaml;
    size_t indent_level = 0;
    bool m_is_root = false;

    std::string indent() const
    {
        return std::string(indent_level * 2, ' ');
    }

    void writeNodeStart(std::string const& p_type, Node const& p_node)
    {
        if (m_is_root)
        {
            yaml << indent() << p_type << ":\n";
            m_is_root = false;
        }
        else
        {
            yaml << indent() << "- " << p_type << ":\n";
        }
        indent_level++;
        yaml << indent() << "_id: " << p_node.id() << "\n";
        yaml << indent() << "name: " << p_node.name << "\n";
    }

    void writeChildrenStart()
    {
        yaml << indent() << "children:\n";
        indent_level++;
    }

    void writeChildrenEnd()
    {
        indent_level--;
    }

    void writeChildStart()
    {
        yaml << indent() << "child:\n";
        indent_level++;
    }

    void writeChildEnd()
    {
        indent_level--;
    }

    void writeNodeEnd()
    {
        indent_level--;
    }

    // Composite nodes
    void visitSequence(Sequence const& p_node) override
    {
        writeNodeStart("Sequence", p_node);
        if (p_node.hasChildren())
        {
            writeChildrenStart();
            for (auto const& child : p_node.getChildren())
            {
                child->accept(*this);
            }
            writeChildrenEnd();
        }
        writeNodeEnd();
    }

    void visitReactiveSequence(ReactiveSequence const& p_node) override
    {
        writeNodeStart("ReactiveSequence", p_node);
        if (p_node.hasChildren())
        {
            writeChildrenStart();
            for (auto const& child : p_node.getChildren())
            {
                child->accept(*this);
            }
            writeChildrenEnd();
        }
        writeNodeEnd();
    }

    void visitSequenceWithMemory(SequenceWithMemory const& p_node) override
    {
        writeNodeStart("SequenceWithMemory", p_node);
        if (p_node.hasChildren())
        {
            writeChildrenStart();
            for (auto const& child : p_node.getChildren())
            {
                child->accept(*this);
            }
            writeChildrenEnd();
        }
        writeNodeEnd();
    }

    void visitSelector(Selector const& p_node) override
    {
        writeNodeStart("Selector", p_node);
        if (p_node.hasChildren())
        {
            writeChildrenStart();
            for (auto const& child : p_node.getChildren())
            {
                child->accept(*this);
            }
            writeChildrenEnd();
        }
        writeNodeEnd();
    }

    void visitReactiveSelector(ReactiveSelector const& p_node) override
    {
        writeNodeStart("ReactiveSelector", p_node);
        if (p_node.hasChildren())
        {
            writeChildrenStart();
            for (auto const& child : p_node.getChildren())
            {
                child->accept(*this);
            }
            writeChildrenEnd();
        }
        writeNodeEnd();
    }

    void visitSelectorWithMemory(SelectorWithMemory const& p_node) override
    {
        writeNodeStart("SelectorWithMemory", p_node);
        if (p_node.hasChildren())
        {
            writeChildrenStart();
            for (auto const& child : p_node.getChildren())
            {
                child->accept(*this);
            }
            writeChildrenEnd();
        }
        writeNodeEnd();
    }

    void visitParallel(Parallel const& p_node) override
    {
        writeNodeStart("Parallel", p_node);
        yaml << indent() << "success_threshold: " << p_node.getMinSuccess()
             << "\n";
        yaml << indent() << "failure_threshold: " << p_node.getMinFail()
             << "\n";
        if (p_node.hasChildren())
        {
            writeChildrenStart();
            for (auto const& child : p_node.getChildren())
            {
                child->accept(*this);
            }
            writeChildrenEnd();
        }
        writeNodeEnd();
    }

    void visitParallelAll(ParallelAll const& p_node) override
    {
        writeNodeStart("Parallel", p_node);
        yaml << indent() << "success_on_all: "
             << (p_node.getSuccessOnAll() ? "true" : "false") << "\n";
        yaml << indent()
             << "fail_on_all: " << (p_node.getFailOnAll() ? "true" : "false")
             << "\n";
        if (p_node.hasChildren())
        {
            writeChildrenStart();
            for (auto const& child : p_node.getChildren())
            {
                child->accept(*this);
            }
            writeChildrenEnd();
        }
        writeNodeEnd();
    }

    // Decorator nodes
    void visitInverter(Inverter const& p_node) override
    {
        writeNodeStart("Inverter", p_node);
        if (p_node.hasChild())
        {
            writeChildStart();
            p_node.getChild().accept(*this);
            writeChildEnd();
        }
        writeNodeEnd();
    }

    void visitRepeater(Repeater const& p_node) override
    {
        writeNodeStart("Repeater", p_node);
        yaml << indent() << "times: " << p_node.getRepetitions() << "\n";
        if (p_node.hasChild())
        {
            writeChildStart();
            p_node.getChild().accept(*this);
            writeChildEnd();
        }
        writeNodeEnd();
    }

    void visitUntilSuccess(UntilSuccess const& p_node) override
    {
        writeNodeStart("UntilSuccess", p_node);
        yaml << indent() << "attempts: " << p_node.getAttempts() << "\n";
        if (p_node.hasChild())
        {
            writeChildStart();
            p_node.getChild().accept(*this);
            writeChildEnd();
        }
        writeNodeEnd();
    }

    void visitUntilFailure(UntilFailure const& p_node) override
    {
        writeNodeStart("UntilFailure", p_node);
        yaml << indent() << "attempts: " << p_node.getAttempts() << "\n";
        if (p_node.hasChild())
        {
            writeChildStart();
            p_node.getChild().accept(*this);
            writeChildEnd();
        }
        writeNodeEnd();
    }

    void visitForceSuccess(ForceSuccess const& p_node) override
    {
        writeNodeStart("ForceSuccess", p_node);
        if (p_node.hasChild())
        {
            writeChildStart();
            p_node.getChild().accept(*this);
            writeChildEnd();
        }
        writeNodeEnd();
    }

    void visitForceFailure(ForceFailure const& p_node) override
    {
        writeNodeStart("ForceFailure", p_node);
        if (p_node.hasChild())
        {
            writeChildStart();
            p_node.getChild().accept(*this);
            writeChildEnd();
        }
        writeNodeEnd();
    }

    void visitTimeout(Timeout const& p_node) override
    {
        writeNodeStart("Timeout", p_node);
        yaml << indent() << "milliseconds: " << p_node.getMilliseconds()
             << "\n";
        if (p_node.hasChild())
        {
            writeChildStart();
            p_node.getChild().accept(*this);
            writeChildEnd();
        }
        writeNodeEnd();
    }

    void visitDelay(Delay const& p_node) override
    {
        writeNodeStart("Delay", p_node);
        yaml << indent() << "milliseconds: " << p_node.getMilliseconds()
             << "\n";
        if (p_node.hasChild())
        {
            writeChildStart();
            p_node.getChild().accept(*this);
            writeChildEnd();
        }
        writeNodeEnd();
    }

    void visitCooldown(Cooldown const& p_node) override
    {
        writeNodeStart("Cooldown", p_node);
        yaml << indent() << "milliseconds: " << p_node.getMilliseconds()
             << "\n";
        if (p_node.hasChild())
        {
            writeChildStart();
            p_node.getChild().accept(*this);
            writeChildEnd();
        }
        writeNodeEnd();
    }

    void visitRunOnce(RunOnce const& p_node) override
    {
        writeNodeStart("RunOnce", p_node);
        if (p_node.hasChild())
        {
            writeChildStart();
            p_node.getChild().accept(*this);
            writeChildEnd();
        }
        writeNodeEnd();
    }

    // Leaf nodes
    void visitSuccess(Success const& p_node) override
    {
        writeNodeStart("Success", p_node);
        writeNodeEnd();
    }

    void visitFailure(Failure const& p_node) override
    {
        writeNodeStart("Failure", p_node);
        writeNodeEnd();
    }

    void visitCondition(Condition const& p_node) override
    {
        writeNodeStart("Condition", p_node);
        writeNodeEnd();
    }

    void visitAction(Action const& p_node) override
    {
        writeNodeStart("Action", p_node);
        writeNodeEnd();
    }

    void visitSugarAction(SugarAction const& p_node) override
    {
        writeNodeStart("Action", p_node);
        writeNodeEnd();
    }

    void visitSubTree(SubTreeNode const& p_node) override
    {
        writeNodeStart("SubTree", p_node);
        if (p_node.handle())
        {
            yaml << indent() << "reference: " << p_node.handle()->id() << "\n";
        }
        writeNodeEnd();
    }

    void visitWait(Wait const& p_node) override
    {
        writeNodeStart("Wait", p_node);
        yaml << indent() << "milliseconds: " << p_node.getMilliseconds()
             << "\n";
        writeNodeEnd();
    }

    void visitSetBlackboard(SetBlackboard const& p_node) override
    {
        writeNodeStart("SetBlackboard", p_node);
        yaml << indent() << "key: " << p_node.getKey() << "\n";
        yaml << indent() << "value: " << p_node.getValue() << "\n";
        writeNodeEnd();
    }

    void visitTree(Tree const& p_tree) override
    {
        yaml << "BehaviorTree:\n";
        indent_level = 1;
        if (p_tree.hasRoot())
        {
            m_is_root = true;
            p_tree.getRoot().accept(*this);
        }
    }
};

// ****************************************************************************
//! \brief Visitor that exports a behavior tree to Mermaid diagram format.
// ****************************************************************************
class MermaidExportVisitor: public ConstBehaviorTreeVisitor
{
public:

    std::stringstream mermaid;
    std::string m_parent_id;

    std::string nodeId(Node const& p_node) const
    {
        return "n" + std::to_string(p_node.id());
    }

    void writeNode(std::string const& p_type,
                   Node const& p_node,
                   std::string const& p_shape_open = "[",
                   std::string const& p_shape_close = "]")
    {
        std::string id = nodeId(p_node);
        std::string label = p_node.name.empty() ? p_type : p_node.name;
        mermaid << "    " << id << p_shape_open << "\"" << p_type << "\\n"
                << label << "\"" << p_shape_close << "\n";

        if (!m_parent_id.empty())
        {
            mermaid << "    " << m_parent_id << " --> " << id << "\n";
        }
    }

    void visitComposite(std::string const& p_type, Composite const& p_node)
    {
        writeNode(p_type, p_node, "[[", "]]");
        std::string prev_parent = m_parent_id;
        m_parent_id = nodeId(p_node);
        for (auto const& child : p_node.getChildren())
        {
            child->accept(*this);
        }
        m_parent_id = prev_parent;
    }

    void visitDecorator(std::string const& p_type, Decorator const& p_node)
    {
        writeNode(p_type, p_node, "{{", "}}");
        std::string prev_parent = m_parent_id;
        m_parent_id = nodeId(p_node);
        if (p_node.hasChild())
        {
            p_node.getChild().accept(*this);
        }
        m_parent_id = prev_parent;
    }

    void visitLeaf(std::string const& p_type, Node const& p_node)
    {
        writeNode(p_type, p_node, "(", ")");
    }

    // Composites
    void visitSequence(Sequence const& p_node) override
    {
        visitComposite("Sequence", p_node);
    }
    void visitReactiveSequence(ReactiveSequence const& p_node) override
    {
        visitComposite("ReactiveSequence", p_node);
    }
    void visitSequenceWithMemory(SequenceWithMemory const& p_node) override
    {
        visitComposite("SequenceWithMemory", p_node);
    }
    void visitSelector(Selector const& p_node) override
    {
        visitComposite("Selector", p_node);
    }
    void visitReactiveSelector(ReactiveSelector const& p_node) override
    {
        visitComposite("ReactiveSelector", p_node);
    }
    void visitSelectorWithMemory(SelectorWithMemory const& p_node) override
    {
        visitComposite("SelectorWithMemory", p_node);
    }
    void visitParallel(Parallel const& p_node) override
    {
        visitComposite("Parallel", p_node);
    }
    void visitParallelAll(ParallelAll const& p_node) override
    {
        visitComposite("Parallel", p_node);
    }

    // Decorators
    void visitInverter(Inverter const& p_node) override
    {
        visitDecorator("Inverter", p_node);
    }
    void visitRepeater(Repeater const& p_node) override
    {
        visitDecorator("Repeater", p_node);
    }
    void visitUntilSuccess(UntilSuccess const& p_node) override
    {
        visitDecorator("UntilSuccess", p_node);
    }
    void visitUntilFailure(UntilFailure const& p_node) override
    {
        visitDecorator("UntilFailure", p_node);
    }
    void visitForceSuccess(ForceSuccess const& p_node) override
    {
        visitDecorator("ForceSuccess", p_node);
    }
    void visitForceFailure(ForceFailure const& p_node) override
    {
        visitDecorator("ForceFailure", p_node);
    }
    void visitTimeout(Timeout const& p_node) override
    {
        visitDecorator("Timeout", p_node);
    }
    void visitDelay(Delay const& p_node) override
    {
        visitDecorator("Delay", p_node);
    }
    void visitCooldown(Cooldown const& p_node) override
    {
        visitDecorator("Cooldown", p_node);
    }
    void visitRunOnce(RunOnce const& p_node) override
    {
        visitDecorator("RunOnce", p_node);
    }

    // Leaves
    void visitSuccess(Success const& p_node) override
    {
        visitLeaf("Success", p_node);
    }
    void visitFailure(Failure const& p_node) override
    {
        visitLeaf("Failure", p_node);
    }
    void visitCondition(Condition const& p_node) override
    {
        visitLeaf("Condition", p_node);
    }
    void visitAction(Action const& p_node) override
    {
        visitLeaf("Action", p_node);
    }
    void visitSugarAction(SugarAction const& p_node) override
    {
        visitLeaf("Action", p_node);
    }
    void visitSubTree(SubTreeNode const& p_node) override
    {
        visitLeaf("SubTree", p_node);
    }
    void visitWait(Wait const& p_node) override
    {
        visitLeaf("Wait", p_node);
    }
    void visitSetBlackboard(SetBlackboard const& p_node) override
    {
        visitLeaf("SetBlackboard", p_node);
    }

    void visitTree(Tree const& tree) override
    {
        mermaid << "flowchart TD\n";
        if (tree.hasRoot())
        {
            tree.getRoot().accept(*this);
        }

        // Add styling
        mermaid << "\n    %% Styling\n";
        mermaid
            << "    classDef composite fill:#4a9eff,stroke:#333,color:#fff\n";
        mermaid
            << "    classDef decorator fill:#ffa500,stroke:#333,color:#fff\n";
        mermaid << "    classDef leaf fill:#90EE90,stroke:#333,color:#333\n";
    }
};

// ----------------------------------------------------------------------------
std::string Exporter::toYAML(Tree const& tree)
{
    std::stringstream result;

    // Export Blackboard if present
    if (tree.blackboard())
    {
        std::string bb_yaml = blackboardToYAML(*tree.blackboard());
        if (!bb_yaml.empty())
        {
            result << bb_yaml << "\n";
        }
    }

    // Export tree structure
    YamlExportVisitor visitor;
    tree.accept(visitor);
    result << visitor.yaml.str();

    return result.str();
}

// ----------------------------------------------------------------------------
bool Exporter::toYAMLFile(Tree const& tree, std::string const& path)
{
    std::ofstream file(path);
    if (!file.is_open())
    {
        return false;
    }

    file << toYAML(tree);
    return file.good();
}

// ----------------------------------------------------------------------------
std::string Exporter::toYAMLStructure(Tree const& tree)
{
    YamlExportVisitor visitor;
    tree.accept(visitor);
    return visitor.yaml.str();
}

// ----------------------------------------------------------------------------
std::string Exporter::blackboardToYAML(Blackboard const& blackboard)
{
    std::stringstream yaml;

    auto keys = blackboard.keys();
    if (keys.empty())
    {
        return "";
    }

    yaml << "Blackboard:\n";
    for (auto const& key : keys)
    {
        auto value = blackboard.get<std::string>(key);
        if (value)
        {
            yaml << "  " << key << ": " << *value << "\n";
        }
    }

    return yaml.str();
}

// ----------------------------------------------------------------------------
std::string Exporter::toMermaid(Tree const& tree)
{
    MermaidExportVisitor visitor;
    tree.accept(visitor);
    return visitor.mermaid.str();
}

} // namespace robotik::bt
