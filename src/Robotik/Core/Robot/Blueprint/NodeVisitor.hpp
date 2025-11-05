/**
 * @file NodeVisitor.hpp
 * @brief Visitor pattern for traversing the node blueprint.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#pragma once

#include <cstddef>

namespace robotik
{

// Forward declarations
class Node;
class Joint;
class Link;
class Geometry;
class Sensor;
class Actuator;
class Frame;

// ****************************************************************************
//! \brief Abstract visitor for traversing and operating on nodes.
//!
//! This visitor pattern allows operations on the node blueprint without
//! polluting node classes with specific algorithms. The visitor maintains
//! its own depth state during traversal.
//!
//! Usage example:
//! \code
//!   class MyVisitor : public NodeVisitor {
//!   public:
//!       void visit(Joint& joint) override {
//!           std::cout << "Joint at depth " << depth() << std::endl;
//!       }
//!       // Implement other visit methods...
//!   };
//!
//!   MyVisitor visitor;
//!   root->traverse(visitor);
//! \endcode
// ****************************************************************************
class NodeVisitor
{
protected:

    //! \brief Current depth in the tree blueprint
    size_t m_depth = 0;

public:

    virtual ~NodeVisitor() = default;

    // ------------------------------------------------------------------------
    //! \brief Visit a Joint node.
    //! \param joint The joint to visit.
    // ------------------------------------------------------------------------
    virtual void visit(Joint& joint) = 0;

    // ------------------------------------------------------------------------
    //! \brief Visit a Link node.
    //! \param link The link to visit.
    // ------------------------------------------------------------------------
    virtual void visit(Link& link) = 0;

    // ------------------------------------------------------------------------
    //! \brief Visit a Geometry node.
    //! \param geometry The geometry to visit.
    // ------------------------------------------------------------------------
    virtual void visit(Geometry& geometry) = 0;

    // ------------------------------------------------------------------------
    //! \brief Visit a Sensor node.
    //! \param sensor The sensor to visit.
    // ------------------------------------------------------------------------
    virtual void visit(Sensor& sensor) = 0;

    // ------------------------------------------------------------------------
    //! \brief Visit an Actuator node.
    //! \param actuator The actuator to visit.
    // ------------------------------------------------------------------------
    virtual void visit(Actuator& actuator) = 0;

    // ------------------------------------------------------------------------
    //! \brief Visit a Frame node.
    //! \param frame The frame to visit.
    // ------------------------------------------------------------------------
    virtual void visit(Frame& frame) = 0;

    // ------------------------------------------------------------------------
    //! \brief Visit a generic Node (fallback).
    //! \param node The node to visit.
    // ------------------------------------------------------------------------
    virtual void visit(Node& node) = 0;

    // ------------------------------------------------------------------------
    //! \brief Get the current depth in the tree.
    //! \return Current depth (0 for root).
    // ------------------------------------------------------------------------
    inline size_t depth() const
    {
        return m_depth;
    }

    // ------------------------------------------------------------------------
    //! \brief Increment depth (called when entering child nodes).
    // ------------------------------------------------------------------------
    inline void incrementDepth()
    {
        ++m_depth;
    }

    // ------------------------------------------------------------------------
    //! \brief Decrement depth (called when leaving child nodes).
    // ------------------------------------------------------------------------
    inline void decrementDepth()
    {
        --m_depth;
    }
};

// ****************************************************************************
//! \brief Const visitor for read-only traversal of the node blueprint.
//!
//! Similar to NodeVisitor but for const operations only.
// ****************************************************************************
class ConstNodeVisitor
{
protected:

    //! \brief Current depth in the tree blueprint
    size_t m_depth = 0;

public:

    virtual ~ConstNodeVisitor() = default;

    // ------------------------------------------------------------------------
    //! \brief Visit a const Joint node.
    //! \param joint The joint to visit.
    // ------------------------------------------------------------------------
    virtual void visit(const Joint& joint) = 0;

    // ------------------------------------------------------------------------
    //! \brief Visit a const Link node.
    //! \param link The link to visit.
    // ------------------------------------------------------------------------
    virtual void visit(const Link& link) = 0;

    // ------------------------------------------------------------------------
    //! \brief Visit a const Geometry node.
    //! \param geometry The geometry to visit.
    // ------------------------------------------------------------------------
    virtual void visit(const Geometry& geometry) = 0;

    // ------------------------------------------------------------------------
    //! \brief Visit a const Sensor node.
    //! \param sensor The sensor to visit.
    // ------------------------------------------------------------------------
    virtual void visit(const Sensor& sensor) = 0;

    // ------------------------------------------------------------------------
    //! \brief Visit a const Actuator node.
    //! \param actuator The actuator to visit.
    // ------------------------------------------------------------------------
    virtual void visit(const Actuator& actuator) = 0;

    // ------------------------------------------------------------------------
    //! \brief Visit a const Frame node.
    //! \param frame The frame to visit.
    // ------------------------------------------------------------------------
    virtual void visit(const Frame& frame) = 0;

    // ------------------------------------------------------------------------
    //! \brief Visit a const generic Node (fallback).
    //! \param node The node to visit.
    // ------------------------------------------------------------------------
    virtual void visit(const Node& node) = 0;

    // ------------------------------------------------------------------------
    //! \brief Get the current depth in the tree.
    //! \return Current depth (0 for root).
    // ------------------------------------------------------------------------
    inline size_t depth() const
    {
        return m_depth;
    }

    // ------------------------------------------------------------------------
    //! \brief Increment depth (called when entering child nodes).
    // ------------------------------------------------------------------------
    inline void incrementDepth()
    {
        ++m_depth;
    }

    // ------------------------------------------------------------------------
    //! \brief Decrement depth (called when leaving child nodes).
    // ------------------------------------------------------------------------
    inline void decrementDepth()
    {
        --m_depth;
    }
};

} // namespace robotik
