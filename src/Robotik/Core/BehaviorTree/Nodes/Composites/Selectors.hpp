/**
 * @file Selectors.hpp
 * @brief Selector composite nodes: Selector, ReactiveSelector,
 * SelectorWithMemory.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 */

#pragma once

#include "Robotik/Core/BehaviorTree/Core/Composite.hpp"

namespace robotik::bt {

// ****************************************************************************
//! \brief The Selector (aka Fallback) composite ticks each child node in order.
//! If a child succeeds or runs, the selector returns the same status.  In the
//! next tick, it will try to run each child in order again.  If all children
//! fails, only then does the selector fail.
// ****************************************************************************
class Selector final: public Composite
{
public:

    // ------------------------------------------------------------------------
    //! \brief Get the string representation of the node type.
    //! \return The string "Selector".
    // ------------------------------------------------------------------------
    [[nodiscard]] static constexpr char const* toString()
    {
        return "Selector";
    }

    // ------------------------------------------------------------------------
    //! \brief Set up the selector.
    //! \return The status of the selector.
    // ------------------------------------------------------------------------
    [[nodiscard]] Status onSetUp() override
    {
        m_iterator = m_children.begin();
        return Status::RUNNING;
    }

    // ------------------------------------------------------------------------
    //! \brief Run the selector.
    //! \return The status of the selector.
    // ------------------------------------------------------------------------
    [[nodiscard]] Status onRunning() override
    {
        while (m_iterator != m_children.end())
        {
            if (Status status = (*m_iterator)->tick();
                status != Status::FAILURE)
            {
                return status;
            }

            m_iterator++;
        }

        return Status::FAILURE;
    }

    void accept(ConstBehaviorTreeVisitor& p_visitor) const override
    {
        p_visitor.visitSelector(*this);
    }
    void accept(BehaviorTreeVisitor& p_visitor) override
    {
        p_visitor.visitSelector(*this);
    }
};

// ****************************************************************************
//! \brief The ReactiveSelector composite ticks each child node in order. If a
//! child succeeds or runs, the reactive selector returns the same status.  In
//! the next tick, it will try to run each child in order again.  If all
//! children fails, only then does the reactive selector fail.
// ****************************************************************************
class ReactiveSelector final: public Composite
{
public:

    // ------------------------------------------------------------------------
    //! \brief Get the string representation of the node type.
    //! \return The string "ReactiveSelector".
    // ------------------------------------------------------------------------
    [[nodiscard]] static constexpr char const* toString()
    {
        return "ReactiveSelector";
    }

    // ------------------------------------------------------------------------
    //! \brief Run the reactive selector.
    //! \return The status of the reactive selector.
    // ------------------------------------------------------------------------
    [[nodiscard]] Status onRunning() override
    {
        m_iterator = m_children.begin();
        while (m_iterator != m_children.end())
        {
            if (Status status = (*m_iterator)->tick();
                status != Status::FAILURE)
            {
                return status;
            }

            m_iterator++;
        }

        return Status::FAILURE;
    }

    void accept(ConstBehaviorTreeVisitor& p_visitor) const override
    {
        p_visitor.visitReactiveSelector(*this);
    }
    void accept(BehaviorTreeVisitor& p_visitor) override
    {
        p_visitor.visitReactiveSelector(*this);
    }
};

// ****************************************************************************
//! \brief The SelectorWithMemory composite ticks each child node in order, and
//! remembers what child it previously tried to tick.  If a child succeeds or
//! runs, the stateful selector returns the same status.  In the next tick, it
//! will try to run the next child or start from the beginning again.  If all
//! children fails, only then does the stateful selector fail.
// ****************************************************************************
class SelectorWithMemory final: public Composite
{
public:

    // ------------------------------------------------------------------------
    //! \brief Get the string representation of the node type.
    //! \return The string "SelectorWithMemory".
    // ------------------------------------------------------------------------
    [[nodiscard]] static constexpr char const* toString()
    {
        return "SelectorWithMemory";
    }

    // ------------------------------------------------------------------------
    //! \brief Run the stateful selector.
    //! \return The status of the stateful selector.
    // ------------------------------------------------------------------------
    [[nodiscard]] Status onRunning() override
    {
        while (m_iterator != m_children.end())
        {
            if (Status status = (*m_iterator)->tick();
                status != Status::FAILURE)
            {
                return status;
            }

            m_iterator++;
        }

        m_iterator = m_children.begin();
        return Status::FAILURE;
    }

    void accept(ConstBehaviorTreeVisitor& p_visitor) const override
    {
        p_visitor.visitSelectorWithMemory(*this);
    }
    void accept(BehaviorTreeVisitor& p_visitor) override
    {
        p_visitor.visitSelectorWithMemory(*this);
    }
};

} // namespace robotik::bt
