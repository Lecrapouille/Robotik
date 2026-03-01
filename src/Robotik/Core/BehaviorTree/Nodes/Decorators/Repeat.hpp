/**
 * @file Repeat.hpp
 * @brief Repeat decorator nodes: Repeater, UntilSuccess, UntilFailure.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 */

#pragma once

#include "Robotik/Core/BehaviorTree/Core/Decorator.hpp"

namespace robotik::bt {

// ****************************************************************************
//! \brief The Repeater decorator repeats its child node a specified number
//! of times (0 = infinite). Unlike BehaviorTree.CPP, this implementation
//! does not use a while loop; the tree engine handles the tick() calls,
//! allowing proper visualization and reactivity between iterations.
//! The decorator ignores the child's SUCCESS/FAILURE status and continues
//! repeating until the limit is reached.
//! The number of repetitions can be read from the blackboard via port
//! remapping.
// ****************************************************************************
class Repeater final: public Decorator
{
public:

    // ------------------------------------------------------------------------
    //! \brief Get the string representation of the node type.
    //! \return The string "Repeater".
    // ------------------------------------------------------------------------
    [[nodiscard]] static constexpr char const* toString()
    {
        return "Repeater";
    }

    // ------------------------------------------------------------------------
    //! \brief Constructor taking a limit of repetitions.
    //! \param[in] p_repetitions The limit of repetitions (0 = infinite).
    // ------------------------------------------------------------------------
    explicit Repeater(size_t p_repetitions = 0)
        : m_default_repetitions(p_repetitions)
    {
        m_type = toString();
    }

    // ------------------------------------------------------------------------
    //! \brief Get the ports provided by the node.
    //! \return The ports provided by the node.
    // ------------------------------------------------------------------------
    [[nodiscard]] PortList providedPorts() const override
    {
        PortList ports;
        ports.addInput<size_t>("repetitions");
        return ports;
    }

    // ------------------------------------------------------------------------
    //! \brief Set up the repeater.
    //! Reads repetitions from blackboard if configured, otherwise uses default.
    //! \return The status of the repeater.
    // ------------------------------------------------------------------------
    [[nodiscard]] Status onSetUp() override
    {
        // Try to read repetitions from blackboard (try int first, then size_t)
        if (auto reps = getInput<int>("repetitions"); reps && *reps >= 0)
        {
            m_repetitions = static_cast<size_t>(*reps);
        }
        else if (auto reps2 = getInput<size_t>("repetitions"); reps2)
        {
            m_repetitions = *reps2;
        }
        else
        {
            m_repetitions = m_default_repetitions;
        }
        m_count = 0;
        return Status::RUNNING;
    }

    // ------------------------------------------------------------------------
    //! \brief Run the repeater.
    //! \return The status of the repeater.
    // ------------------------------------------------------------------------
    [[nodiscard]] Status onRunning() override
    {
        Status status = m_child->tick();

        if (status == Status::RUNNING)
        {
            return Status::RUNNING;
        }

        // Child finished (SUCCESS or FAILURE), reset for next iteration
        m_child->reset();

        // Increment count and saturate to m_repetitions if limited
        if (m_repetitions > 0)
        {
            ++m_count;
            if (m_count >= m_repetitions)
            {
                m_count = m_repetitions;
                return Status::SUCCESS;
            }
        }

        // Continue (infinite or more iterations needed)
        return Status::RUNNING;
    }

    // ------------------------------------------------------------------------
    //! \brief Get the current count of repetitions.
    // ------------------------------------------------------------------------
    [[nodiscard]] size_t getCount() const
    {
        return m_count;
    }

    // ------------------------------------------------------------------------
    //! \brief Get the limit number of repetitions.
    // ------------------------------------------------------------------------
    [[nodiscard]] size_t getRepetitions() const
    {
        return m_repetitions;
    }

    void accept(ConstBehaviorTreeVisitor& p_visitor) const override
    {
        p_visitor.visitRepeater(*this);
    }
    void accept(BehaviorTreeVisitor& p_visitor) override
    {
        p_visitor.visitRepeater(*this);
    }

private:

    size_t m_count = 0;
    size_t m_default_repetitions;
    size_t m_repetitions = 0;
};

// Backward compatibility alias
using Repeat = Repeater;

// ****************************************************************************
//! \brief The UntilSuccess decorator repeats until the child returns success
//! and then returns success. Unlike BehaviorTree.CPP, this implementation
//! does not use a while loop; the tree engine handles the tick() calls,
//! allowing proper visualization and reactivity between iterations.
//! The decorator can optionally limit the number of attempts (0 = infinite).
// ****************************************************************************
class UntilSuccess final: public Decorator
{
public:

    // ------------------------------------------------------------------------
    //! \brief Get the string representation of the node type.
    //! \return The string "UntilSuccess".
    // ------------------------------------------------------------------------
    [[nodiscard]] static constexpr char const* toString()
    {
        return "UntilSuccess";
    }

    // ------------------------------------------------------------------------
    //! \brief Constructor taking an optional limit of attempts.
    //! \param[in] p_attempts The maximum number of attempts (0 = infinite).
    // ------------------------------------------------------------------------
    explicit UntilSuccess(size_t p_attempts = 0) : m_attempts(p_attempts) {}

    // ------------------------------------------------------------------------
    //! \brief Set up the until success decorator.
    //! \return The status of the decorator.
    // ------------------------------------------------------------------------
    [[nodiscard]] Status onSetUp() override
    {
        m_count = 0;
        return Status::RUNNING;
    }

    // ------------------------------------------------------------------------
    //! \brief Run the until success decorator.
    //! \return The status of the decorator.
    // ------------------------------------------------------------------------
    [[nodiscard]] Status onRunning() override
    {
        Status status = m_child->tick();

        if (status == Status::SUCCESS)
        {
            return Status::SUCCESS;
        }
        else if (status == Status::RUNNING)
        {
            return Status::RUNNING;
        }

        // Child failed, reset for next iteration
        m_child->reset();

        // Increment count and check limit if attempts are limited
        if (m_attempts > 0)
        {
            ++m_count;
            if (m_count >= m_attempts)
            {
                m_count = m_attempts; // Saturate
                return Status::FAILURE;
            }
        }

        // Continue (infinite or more attempts needed)
        return Status::RUNNING;
    }

    // ------------------------------------------------------------------------
    //! \brief Get the current count of attempts.
    //! \return The current count of attempts.
    // ------------------------------------------------------------------------
    [[nodiscard]] size_t getCount() const
    {
        return m_count;
    }

    // ------------------------------------------------------------------------
    //! \brief Get the limit number of attempts.
    //! \return The limit number of attempts (0 = infinite).
    // ------------------------------------------------------------------------
    [[nodiscard]] size_t getAttempts() const
    {
        return m_attempts;
    }

    void accept(ConstBehaviorTreeVisitor& p_visitor) const override
    {
        p_visitor.visitUntilSuccess(*this);
    }
    void accept(BehaviorTreeVisitor& p_visitor) override
    {
        p_visitor.visitUntilSuccess(*this);
    }

private:

    size_t m_count = 0;
    size_t m_attempts;
};

// ****************************************************************************
//! \brief The UntilFailure decorator repeats until the child returns failure
//! and then returns success. Unlike BehaviorTree.CPP, this implementation
//! does not use a while loop; the tree engine handles the tick() calls,
//! allowing proper visualization and reactivity between iterations.
//! The decorator can optionally limit the number of attempts (0 = infinite).
// ****************************************************************************
class UntilFailure final: public Decorator
{
public:

    // ------------------------------------------------------------------------
    //! \brief Get the string representation of the node type.
    //! \return The string "UntilFailure".
    // ------------------------------------------------------------------------
    [[nodiscard]] static constexpr char const* toString()
    {
        return "UntilFailure";
    }

    // ------------------------------------------------------------------------
    //! \brief Constructor taking an optional limit of attempts.
    //! \param[in] p_attempts The maximum number of attempts (0 = infinite).
    // ------------------------------------------------------------------------
    explicit UntilFailure(size_t p_attempts = 0) : m_attempts(p_attempts) {}

    // ------------------------------------------------------------------------
    //! \brief Set up the until failure decorator.
    //! \return The status of the decorator.
    // ------------------------------------------------------------------------
    [[nodiscard]] Status onSetUp() override
    {
        m_count = 0;
        return Status::RUNNING;
    }

    // ------------------------------------------------------------------------
    //! \brief Execute the decorator.
    //! \return The status of the decorator.
    // ------------------------------------------------------------------------
    [[nodiscard]] Status onRunning() override
    {
        Status status = m_child->tick();

        if (status == Status::FAILURE)
        {
            return Status::SUCCESS;
        }
        else if (status == Status::RUNNING)
        {
            return Status::RUNNING;
        }

        // Child succeeded, reset for next iteration
        m_child->reset();

        // Increment count and check limit if attempts are limited
        if (m_attempts > 0)
        {
            ++m_count;
            if (m_count >= m_attempts)
            {
                m_count = m_attempts; // Saturate
                return Status::FAILURE;
            }
        }

        // Continue (infinite or more attempts needed)
        return Status::RUNNING;
    }

    // ------------------------------------------------------------------------
    //! \brief Get the current count of attempts.
    //! \return The current count of attempts.
    // ------------------------------------------------------------------------
    [[nodiscard]] size_t getCount() const
    {
        return m_count;
    }

    // ------------------------------------------------------------------------
    //! \brief Get the limit number of attempts.
    //! \return The limit number of attempts (0 = infinite).
    // ------------------------------------------------------------------------
    [[nodiscard]] size_t getAttempts() const
    {
        return m_attempts;
    }

    void accept(ConstBehaviorTreeVisitor& p_visitor) const override
    {
        p_visitor.visitUntilFailure(*this);
    }
    void accept(BehaviorTreeVisitor& p_visitor) override
    {
        p_visitor.visitUntilFailure(*this);
    }

private:

    size_t m_count = 0;
    size_t m_attempts;
};

} // namespace robotik::bt
