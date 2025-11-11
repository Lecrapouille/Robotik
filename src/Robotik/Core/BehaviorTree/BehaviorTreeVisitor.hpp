/**
 * @file BehaviorTreeVisitor.hpp
 * @brief Visitor interface for behavior tree nodes.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#pragma once

namespace bt
{

// Forward declarations
class Tree;
class Sequence;
class ReactiveSequence;
class StatefulSequence;
class Selector;
class ReactiveSelector;
class StatefulSelector;
class Parallel;
class ParallelAll;
class Inverter;
class Retry;
class Repeat;
class UntilSuccess;
class UntilFailure;
class ForceSuccess;
class ForceFailure;
class Success;
class Failure;
class Condition;
class Action;
class SugarAction;

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
//!       void visitSequence(Sequence const& node) override {
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
    virtual void visitSequence(Sequence const& node) = 0;
    virtual void visitReactiveSequence(ReactiveSequence const& node) = 0;
    virtual void visitStatefulSequence(StatefulSequence const& node) = 0;
    virtual void visitSelector(Selector const& node) = 0;
    virtual void visitReactiveSelector(ReactiveSelector const& node) = 0;
    virtual void visitStatefulSelector(StatefulSelector const& node) = 0;
    virtual void visitParallel(Parallel const& node) = 0;
    virtual void visitParallelAll(ParallelAll const& node) = 0;

    // Decorator nodes
    virtual void visitInverter(Inverter const& node) = 0;
    virtual void visitRetry(Retry const& node) = 0;
    virtual void visitRepeat(Repeat const& node) = 0;
    virtual void visitUntilSuccess(UntilSuccess const& node) = 0;
    virtual void visitUntilFailure(UntilFailure const& node) = 0;
    virtual void visitForceSuccess(ForceSuccess const& node) = 0;
    virtual void visitForceFailure(ForceFailure const& node) = 0;

    // Leaf nodes
    virtual void visitSuccess(Success const& node) = 0;
    virtual void visitFailure(Failure const& node) = 0;
    virtual void visitCondition(Condition const& node) = 0;
    virtual void visitAction(Action const& node) = 0;
    virtual void visitSugarAction(SugarAction const& node) = 0;

    // Tree node
    virtual void visitTree(Tree const& node) = 0;
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
//!       void visitSequence(Sequence& node) override {
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
    virtual void visitSequence(Sequence& node) = 0;
    virtual void visitReactiveSequence(ReactiveSequence& node) = 0;
    virtual void visitStatefulSequence(StatefulSequence& node) = 0;
    virtual void visitSelector(Selector& node) = 0;
    virtual void visitReactiveSelector(ReactiveSelector& node) = 0;
    virtual void visitStatefulSelector(StatefulSelector& node) = 0;
    virtual void visitParallel(Parallel& node) = 0;
    virtual void visitParallelAll(ParallelAll& node) = 0;

    // Decorator nodes
    virtual void visitInverter(Inverter& node) = 0;
    virtual void visitRetry(Retry& node) = 0;
    virtual void visitRepeat(Repeat& node) = 0;
    virtual void visitUntilSuccess(UntilSuccess& node) = 0;
    virtual void visitUntilFailure(UntilFailure& node) = 0;
    virtual void visitForceSuccess(ForceSuccess& node) = 0;
    virtual void visitForceFailure(ForceFailure& node) = 0;

    // Leaf nodes
    virtual void visitSuccess(Success& node) = 0;
    virtual void visitFailure(Failure& node) = 0;
    virtual void visitCondition(Condition& node) = 0;
    virtual void visitAction(Action& node) = 0;
    virtual void visitSugarAction(SugarAction& node) = 0;

    // Tree node
    virtual void visitTree(Tree& node) = 0;
};

} // namespace bt
