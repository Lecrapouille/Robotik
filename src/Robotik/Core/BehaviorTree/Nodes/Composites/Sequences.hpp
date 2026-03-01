/**
 * @file Sequences.hpp
 * @brief Sequence composite nodes: Sequence, ReactiveSequence,
 * SequenceWithMemory.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 */

#pragma once

#include "Robotik/Core/BehaviorTree/Core/Composite.hpp"

namespace robotik::bt {

// ****************************************************************************
//! \brief The Sequence composite ticks each child node in order. If a child
//! fails or runs, the sequence returns the same status.  In the next tick, it
//! will try to run each child in order again.  If all children succeeds, only
//! then does the sequence succeed.
// ****************************************************************************
class Sequence: public Composite
{
public:

    // ------------------------------------------------------------------------
    //! \brief Get the string representation of the node type.
    //! \return The string "Sequence".
    // ------------------------------------------------------------------------
    [[nodiscard]] static constexpr char const* toString()
    {
        return "Sequence";
    }

    // ------------------------------------------------------------------------
    //! \brief Set up the sequence.
    //! \return The status of the sequence.
    // ------------------------------------------------------------------------
    [[nodiscard]] Status onSetUp() override
    {
        m_iterator = m_children.begin();
        return Status::RUNNING;
    }

    // ------------------------------------------------------------------------
    //! \brief Run the sequence.
    //! \return The status of the sequence.
    // ------------------------------------------------------------------------
    [[nodiscard]] Status onRunning() override
    {
        while (m_iterator != m_children.end())
        {
            if (auto status = (*m_iterator)->tick(); status != Status::SUCCESS)
            {
                return status;
            }

            m_iterator++;
        }

        return Status::SUCCESS;
    }

    // ------------------------------------------------------------------------
    //! \brief Accept a visitor.
    // ------------------------------------------------------------------------
    void accept(ConstBehaviorTreeVisitor& p_visitor) const override
    {
        p_visitor.visitSequence(*this);
    }
    void accept(BehaviorTreeVisitor& p_visitor) override
    {
        p_visitor.visitSequence(*this);
    }
};

// ****************************************************************************
//! \brief The ReactiveSequence composite ticks each child node in order. If a
//! child fails or runs, the reactive sequence returns the same status.  In the
//! next tick, it will try to run each child in order again.  If all children
//! succeeds, only then does the reactive sequence succeed.
// ****************************************************************************
class ReactiveSequence final: public Composite
{
public:

    // ------------------------------------------------------------------------
    //! \brief Get the string representation of the node type.
    //! \return The string "ReactiveSequence".
    // ------------------------------------------------------------------------
    [[nodiscard]] static constexpr char const* toString()
    {
        return "ReactiveSequence";
    }

    // ------------------------------------------------------------------------
    //! \brief Run the reactive sequence.
    //! \return The status of the reactive sequence.
    // ------------------------------------------------------------------------
    [[nodiscard]] Status onRunning() override
    {
        m_iterator = m_children.begin();
        while (m_iterator != m_children.end())
        {
            if (auto status = (*m_iterator)->tick(); status != Status::SUCCESS)
            {
                return status;
            }

            m_iterator++;
        }

        return Status::SUCCESS;
    }

    void accept(ConstBehaviorTreeVisitor& p_visitor) const override
    {
        p_visitor.visitReactiveSequence(*this);
    }
    void accept(BehaviorTreeVisitor& p_visitor) override
    {
        p_visitor.visitReactiveSequence(*this);
    }
};

// ****************************************************************************
//! \brief The SequenceWithMemory composite ticks each child node in order, and
//! remembers what child it previously tried to tick.  If a child fails or runs,
//! the stateful sequence returns the same status.  In the next tick, it will
//! try to run the next child or start from the beginning again.  If all
//! children succeeds, only then does the stateful sequence succeed.
// ****************************************************************************
class SequenceWithMemory final: public Composite
{
public:

    // ------------------------------------------------------------------------
    //! \brief Get the string representation of the node type.
    //! \return The string "SequenceWithMemory".
    // ------------------------------------------------------------------------
    [[nodiscard]] static constexpr char const* toString()
    {
        return "SequenceWithMemory";
    }

    // ------------------------------------------------------------------------
    //! \brief Run the stateful sequence.
    //! \return The status of the stateful sequence.
    // ------------------------------------------------------------------------
    [[nodiscard]] Status onRunning() override
    {
        while (m_iterator != m_children.end())
        {
            if (auto status = (*m_iterator)->tick(); status != Status::SUCCESS)
            {
                return status;
            }

            m_iterator++;
        }

        m_iterator = m_children.begin();
        return Status::SUCCESS;
    }

    void accept(ConstBehaviorTreeVisitor& p_visitor) const override
    {
        p_visitor.visitSequenceWithMemory(*this);
    }
    void accept(BehaviorTreeVisitor& p_visitor) override
    {
        p_visitor.visitSequenceWithMemory(*this);
    }
};

} // namespace robotik::bt
