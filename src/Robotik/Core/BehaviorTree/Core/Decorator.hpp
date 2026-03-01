/**
 * @file Decorator.hpp
 * @brief Base class for decorator nodes that can have only one child.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 */

#pragma once

#include "Robotik/Core/BehaviorTree/Core/Node.hpp"

namespace robotik::bt {

// ****************************************************************************
//! \brief Base class for decorator nodes that can have only one child.
//! Decorator nodes are used to modify the behavior of their child node.
// ****************************************************************************
class Decorator: public Node
{
public:

    // ------------------------------------------------------------------------
    //! \brief Set the child node of the decorator.
    //! \param[in] child The child node to set.
    // ------------------------------------------------------------------------
    void setChild(Node::Ptr p_child)
    {
        m_child = std::move(p_child);
    }

    // ------------------------------------------------------------------------
    //! \brief Set the child node of the decorator.
    //! \param[in] args The arguments to pass to the constructor of T
    //! \return A reference to the new child node
    // ------------------------------------------------------------------------
    template <class T, typename... Args>
    [[nodiscard]] inline T& createChild(Args&&... p_args)
    {
        auto child = Node::create<T>(std::forward<Args>(p_args)...);
        T* ptr = child.get();
        m_child = std::move(child);
        return *ptr;
    }

    // ------------------------------------------------------------------------
    //! \brief Check if the decorator has a child node.
    //! \return True if the decorator has a child node, false otherwise.
    // ------------------------------------------------------------------------
    [[nodiscard]] inline bool hasChild() const
    {
        return m_child != nullptr;
    }

    // ------------------------------------------------------------------------
    //! \brief Get the child node of the decorator (const version).
    //! \note The child node shall exist. Use hasChild() to check.
    //! \return The child node reference.
    // ------------------------------------------------------------------------
    [[nodiscard]] inline Node const& getChild() const
    {
        return *(m_child.get());
    }

    // ------------------------------------------------------------------------
    //! \brief Get the child node of the decorator (non-const version).
    //! \note The child node shall exist. Use hasChild() to check.
    //! \return The child node reference.
    // ------------------------------------------------------------------------
    [[nodiscard]] inline Node& getChild()
    {
        return *(m_child.get());
    }

    // ------------------------------------------------------------------------
    //! \brief Check if the composite node is valid.
    //! \return True if the composite node is valid, false otherwise.
    // ------------------------------------------------------------------------
    [[nodiscard]] bool isValid() const override
    {
        return (m_child != nullptr) && (m_child->isValid());
    }

    // ------------------------------------------------------------------------
    //! \brief Reset the decorator and its child recursively.
    // ------------------------------------------------------------------------
    void reset() override
    {
        if (m_child != nullptr)
        {
            m_child->reset();
        }
        m_status = Status::INVALID;
    }

    // ------------------------------------------------------------------------
    //! \brief Halt the decorator and its child recursively.
    // ------------------------------------------------------------------------
    void halt() override
    {
        if (m_child != nullptr)
        {
            m_child->halt();
        }
        if (m_status == Status::RUNNING)
        {
            onHalt();
        }
        m_status = Status::INVALID;
    }

protected:

    Node::Ptr m_child = nullptr;
};

} // namespace robotik::bt
