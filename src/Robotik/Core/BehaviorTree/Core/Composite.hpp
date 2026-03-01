/**
 * @file Composite.hpp
 * @brief Base class for composite nodes that can have multiple children.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 */

#pragma once

#include "Robotik/Core/BehaviorTree/Core/Node.hpp"

#include <algorithm>
#include <vector>

namespace robotik::bt {

// ****************************************************************************
//! \brief Base class for composite nodes that can have multiple children.
//! Composite nodes are used to control the flow of the behavior tree.
// ****************************************************************************
class Composite: public Node
{
public:

    // ------------------------------------------------------------------------
    //! \brief Initialize the composite node before running.
    //! \return Status::RUNNING to proceed with onRunning().
    // ------------------------------------------------------------------------
    [[nodiscard]] Status onSetUp() override
    {
        m_iterator = m_children.begin();
        return Status::RUNNING;
    }

    // ------------------------------------------------------------------------
    //! \brief Add an existing node as a child.
    //! \param[in] p_child Pointer to the child node to add.
    // ------------------------------------------------------------------------
    void addChild(Node::Ptr p_child)
    {
        m_children.emplace_back(std::move(p_child));
    }

    // ------------------------------------------------------------------------
    //! \brief Create and add a new child node of type T.
    //! \param[in] p_args The arguments to pass to the constructor of T.
    //! \return A reference to the new child node.
    // ------------------------------------------------------------------------
    template <class T, typename... Args>
    [[nodiscard]] inline T& addChild(Args&&... p_args)
    {
        static_assert(std::is_base_of_v<Node, T>, "T must inherit from Node");
        auto child = Node::create<T>(std::forward<Args>(p_args)...);
        T* ptr = child.get();
        m_children.emplace_back(std::move(child));
        return *ptr;
    }

    // ------------------------------------------------------------------------
    //! \brief Check if the composite node has children.
    //! \return True if the composite node has children, false otherwise.
    // ------------------------------------------------------------------------
    [[nodiscard]] inline bool hasChildren() const
    {
        return !m_children.empty();
    }

    // ------------------------------------------------------------------------
    //! \brief Get the children nodes (const version)
    //! \return Const reference to the vector of child nodes
    // ------------------------------------------------------------------------
    [[nodiscard]] std::vector<Node::Ptr> const& getChildren() const
    {
        return m_children;
    }

    // ------------------------------------------------------------------------
    //! \brief Get the children nodes (non-const version)
    //! \return Reference to the vector of child nodes
    // ------------------------------------------------------------------------
    [[nodiscard]] std::vector<Node::Ptr>& getChildren()
    {
        return m_children;
    }

    // ------------------------------------------------------------------------
    //! \brief Check if the composite node is valid.
    //! \return True if the composite node is valid, false otherwise.
    // ------------------------------------------------------------------------
    [[nodiscard]] bool isValid() const override
    {
        if (m_children.empty())
        {
            return false;
        }

        return std::all_of(m_children.begin(),
                           m_children.end(),
                           [](const auto& child) { return child->isValid(); });
    }

    // ------------------------------------------------------------------------
    //! \brief Reset the composite and all its children recursively.
    // ------------------------------------------------------------------------
    void reset() override
    {
        for (auto const& child : m_children)
        {
            child->reset();
        }
        m_iterator = m_children.begin();
        m_status = Status::INVALID;
    }

    // ------------------------------------------------------------------------
    //! \brief Halt the composite and all its children recursively.
    // ------------------------------------------------------------------------
    void halt() override
    {
        for (auto const& child : m_children)
        {
            child->halt();
        }
        if (m_status == Status::RUNNING)
        {
            onHalt();
        }
        m_status = Status::INVALID;
    }

protected:

    //! \brief The children nodes of the composite node.
    std::vector<Node::Ptr> m_children;
    //! \brief The iterator to the current child node.
    std::vector<Node::Ptr>::iterator m_iterator;
};

} // namespace robotik::bt
