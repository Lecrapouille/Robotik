/**
 * @file MermaidExporter.hpp
 * @brief Mermaid diagram exporter for behavior trees.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#pragma once

#include "Robotik/Core/BehaviorTree/BehaviorTreeVisitor.hpp"
#include "Robotik/Core/BehaviorTree/Exporters/BehaviorTreeExporter.hpp"

#include <sstream>

namespace bt
{

// Forward declaration
class Node;

// ****************************************************************************
//! \brief Mermaid diagram exporter for behavior trees.
//! Uses the Visitor pattern to traverse and export the tree structure.
// ****************************************************************************
class MermaidExporter final: public BehaviorTreeExporter,
                             public ConstBehaviorTreeVisitor
{
public:

    // ------------------------------------------------------------------------
    //! \brief Export a behavior tree to a Mermaid file.
    //! \param tree The tree to export.
    //! \param filename Path to the output Mermaid file.
    //! \return True if successful, false otherwise.
    // ------------------------------------------------------------------------
    bool exportTo(const Tree& tree, const std::string& filename) override;

    // ------------------------------------------------------------------------
    //! \brief Export a behavior tree to a Mermaid string.
    //! \param tree The tree to export.
    //! \return Mermaid string representation of the tree.
    // ------------------------------------------------------------------------
    std::string toString(const Tree& tree) override;

    // ------------------------------------------------------------------------
    //! \brief Get the last error message.
    //! \return Error message or empty string if no error.
    // ------------------------------------------------------------------------
    std::string error() const override
    {
        return m_error;
    }

    // ConstBehaviorTreeVisitor implementation
    void visitTree(Tree const& node) override;
    void visitSequence(Sequence const& node) override;
    void visitReactiveSequence(ReactiveSequence const& node) override;
    void visitStatefulSequence(StatefulSequence const& node) override;
    void visitSelector(Selector const& node) override;
    void visitReactiveSelector(ReactiveSelector const& node) override;
    void visitStatefulSelector(StatefulSelector const& node) override;
    void visitParallel(Parallel const& node) override;
    void visitParallelAll(ParallelAll const& node) override;
    void visitInverter(Inverter const& node) override;
    void visitRetry(Retry const& node) override;
    void visitRepeat(Repeat const& node) override;
    void visitUntilSuccess(UntilSuccess const& node) override;
    void visitUntilFailure(UntilFailure const& node) override;
    void visitForceSuccess(ForceSuccess const& node) override;
    void visitForceFailure(ForceFailure const& node) override;
    void visitSuccess(Success const& node) override;
    void visitFailure(Failure const& node) override;
    void visitCondition(Condition const& node) override;
    void visitSugarAction(SugarAction const& node) override;

private:

    void generateNodeHelper(Node const* node,
                            size_t parent_id,
                            std::string const& node_class);

    std::string m_error;
    std::stringstream m_result;
    size_t m_counter = 0;
    size_t m_parent_id = 0;
};

} // namespace bt
