/**
 * @file BehaviorTreeVisitor.hpp
 * @brief Visitor interface for behavior tree nodes.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#pragma once

namespace robotik::bt {

// Forward declarations
class Tree;
class SubTreeNode;
class Sequence;
class ReactiveSequence;
class SequenceWithMemory;
class Selector;
class ReactiveSelector;
class SelectorWithMemory;
class Parallel;
class ParallelAll;
class Inverter;
class Repeater;
using Repeat = Repeater; // Backward compatibility
class UntilSuccess;
class UntilFailure;
class ForceSuccess;
class ForceFailure;
class Timeout;
class Delay;
class Cooldown;
class RunOnce;
class Success;
class Failure;
class Condition;
class Action;
class SugarAction;
class Wait;
class SetBlackboard;

// ****************************************************************************
//! \brief Const visitor interface for behavior tree nodes (read-only).
//! This enables operations like export, rendering, and analysis without
//! modifying the node classes themselves.
//!
//! Use this visitor when you only need to read/inspect the tree structure.
//!
//! Example usage:
//! \code
//!   class MyExporter : public ConstBehaviorTreeVisitor {
//!   public:
//!       void visitSequence(Sequence const& p_node) override {
//!           // Read node data
//!       }
//!       // ... implement other visit methods
//!   };
//! \endcode
// ****************************************************************************
class ConstBehaviorTreeVisitor
{
public:

    virtual ~ConstBehaviorTreeVisitor() = default;

    // Composite nodes
    virtual void visitSequence(Sequence const& p_node) = 0;
    virtual void visitReactiveSequence(ReactiveSequence const& p_node) = 0;
    virtual void visitSequenceWithMemory(SequenceWithMemory const& p_node) = 0;
    virtual void visitSelector(Selector const& p_node) = 0;
    virtual void visitReactiveSelector(ReactiveSelector const& p_node) = 0;
    virtual void visitSelectorWithMemory(SelectorWithMemory const& p_node) = 0;
    virtual void visitParallel(Parallel const& p_node) = 0;
    virtual void visitParallelAll(ParallelAll const& p_node) = 0;

    // Decorator nodes
    virtual void visitInverter(Inverter const& p_node) = 0;
    virtual void visitRepeater(Repeater const& p_node) = 0;
    virtual void visitUntilSuccess(UntilSuccess const& p_node) = 0;
    virtual void visitUntilFailure(UntilFailure const& p_node) = 0;
    virtual void visitForceSuccess(ForceSuccess const& p_node) = 0;
    virtual void visitForceFailure(ForceFailure const& p_node) = 0;
    virtual void visitTimeout(Timeout const& p_node) = 0;
    virtual void visitDelay(Delay const& p_node) = 0;
    virtual void visitCooldown(Cooldown const& p_node) = 0;
    virtual void visitRunOnce(RunOnce const& p_node) = 0;

    // Leaf nodes
    virtual void visitSuccess(Success const& p_node) = 0;
    virtual void visitFailure(Failure const& p_node) = 0;
    virtual void visitCondition(Condition const& p_node) = 0;
    virtual void visitAction(Action const& p_node) = 0;
    virtual void visitSugarAction(SugarAction const& p_node) = 0;
    virtual void visitSubTree(SubTreeNode const& p_node) = 0;
    virtual void visitWait(Wait const& p_node) = 0;
    virtual void visitSetBlackboard(SetBlackboard const& p_node) = 0;

    // Tree node
    virtual void visitTree(Tree const& p_node) = 0;
};

// ****************************************************************************
//! \brief Non-const visitor interface for behavior tree nodes (read-write).
//! This enables operations that may modify the tree structure.
//!
//! Use this visitor when you need to modify nodes during traversal.
//!
//! Example usage:
//! \code
//!   class MyModifier : public BehaviorTreeVisitor {
//!   public:
//!       void visitSequence(Sequence& p_node) override {
//!           // Modify node
//!           node.name = "Modified";
//!       }
//!       // ... implement other visit methods
//!   };
//! \endcode
// ****************************************************************************
class BehaviorTreeVisitor
{
public:

    virtual ~BehaviorTreeVisitor() = default;

    // Composite nodes
    virtual void visitSequence(Sequence& p_node) = 0;
    virtual void visitReactiveSequence(ReactiveSequence& p_node) = 0;
    virtual void visitSequenceWithMemory(SequenceWithMemory& p_node) = 0;
    virtual void visitSelector(Selector& p_node) = 0;
    virtual void visitReactiveSelector(ReactiveSelector& p_node) = 0;
    virtual void visitSelectorWithMemory(SelectorWithMemory& p_node) = 0;
    virtual void visitParallel(Parallel& p_node) = 0;
    virtual void visitParallelAll(ParallelAll& p_node) = 0;

    // Decorator nodes
    virtual void visitInverter(Inverter& p_node) = 0;
    virtual void visitRepeater(Repeater& p_node) = 0;
    virtual void visitUntilSuccess(UntilSuccess& p_node) = 0;
    virtual void visitUntilFailure(UntilFailure& p_node) = 0;
    virtual void visitForceSuccess(ForceSuccess& p_node) = 0;
    virtual void visitForceFailure(ForceFailure& p_node) = 0;
    virtual void visitTimeout(Timeout& p_node) = 0;
    virtual void visitDelay(Delay& p_node) = 0;
    virtual void visitCooldown(Cooldown& p_node) = 0;
    virtual void visitRunOnce(RunOnce& p_node) = 0;

    // Leaf nodes
    virtual void visitSuccess(Success& p_node) = 0;
    virtual void visitFailure(Failure& p_node) = 0;
    virtual void visitCondition(Condition& p_node) = 0;
    virtual void visitAction(Action& p_node) = 0;
    virtual void visitSugarAction(SugarAction& p_node) = 0;
    virtual void visitSubTree(SubTreeNode& p_node) = 0;
    virtual void visitWait(Wait& p_node) = 0;
    virtual void visitSetBlackboard(SetBlackboard& p_node) = 0;

    // Tree node
    virtual void visitTree(Tree& p_node) = 0;
};

} // namespace robotik::bt
