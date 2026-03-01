/**
 * @file Logical.hpp
 * @brief Logical decorator nodes: Inverter, ForceSuccess, ForceFailure,
 * RunOnce.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 */

#pragma once

#include "Robotik/Core/BehaviorTree/Core/Decorator.hpp"

namespace robotik::bt {

// ****************************************************************************
//! \brief The ForceSuccess decorator returns RUNNING if the child is RUNNING,
//! else returns SUCCESS, regardless of what happens to the child.
// ****************************************************************************
class ForceSuccess final: public Decorator
{
public:

    // ------------------------------------------------------------------------
    //! \brief Get the string representation of the node type.
    //! \return The string "ForceSuccess".
    // ------------------------------------------------------------------------
    [[nodiscard]] static constexpr char const* toString()
    {
        return "ForceSuccess";
    }

    // ------------------------------------------------------------------------
    //! \brief Run the succeeder.
    //! \return The status of the succeeder.
    // ------------------------------------------------------------------------
    [[nodiscard]] Status onRunning() override
    {
        Status status = m_child->tick();
        return (status == Status::RUNNING) ? Status::RUNNING : Status::SUCCESS;
    }

    void accept(ConstBehaviorTreeVisitor& p_visitor) const override
    {
        p_visitor.visitForceSuccess(*this);
    }
    void accept(BehaviorTreeVisitor& p_visitor) override
    {
        p_visitor.visitForceSuccess(*this);
    }
};

// ****************************************************************************
//! \brief The ForceFailure decorator returns RUNNING if the child is RUNNING,
//! else returns FAILURE, regardless of what happens to the child.
// ****************************************************************************
class ForceFailure final: public Decorator
{
public:

    // ------------------------------------------------------------------------
    //! \brief Get the string representation of the node type.
    //! \return The string "ForceFailure".
    // ------------------------------------------------------------------------
    [[nodiscard]] static constexpr char const* toString()
    {
        return "ForceFailure";
    }

    // ------------------------------------------------------------------------
    //! \brief Run the failer.
    //! \return The status of the failer.
    // ------------------------------------------------------------------------
    [[nodiscard]] Status onRunning() override
    {
        Status status = m_child->tick();
        return (status == Status::RUNNING) ? Status::RUNNING : Status::FAILURE;
    }

    void accept(ConstBehaviorTreeVisitor& p_visitor) const override
    {
        p_visitor.visitForceFailure(*this);
    }
    void accept(BehaviorTreeVisitor& p_visitor) override
    {
        p_visitor.visitForceFailure(*this);
    }
};

// ****************************************************************************
//! \brief The Inverter decorator returns RUNNING if the child is RUNNING,
//! else returns the opposite of the child's status, i.e. FAILURE becomes
//! SUCCESS and SUCCESS becomes FAILURE.
// ****************************************************************************
class Inverter final: public Decorator
{
public:

    // ------------------------------------------------------------------------
    //! \brief Get the string representation of the node type.
    //! \return The string "Inverter".
    // ------------------------------------------------------------------------
    [[nodiscard]] static constexpr char const* toString()
    {
        return "Inverter";
    }

    // ------------------------------------------------------------------------
    //! \brief Run the inverter.
    //! \return The status of the inverter.
    // ------------------------------------------------------------------------
    [[nodiscard]] Status onRunning() override
    {
        auto s = m_child->tick();
        if (s == Status::SUCCESS)
        {
            return Status::FAILURE;
        }
        else if (s == Status::FAILURE)
        {
            return Status::SUCCESS;
        }

        return s;
    }

    void accept(ConstBehaviorTreeVisitor& p_visitor) const override
    {
        p_visitor.visitInverter(*this);
    }
    void accept(BehaviorTreeVisitor& p_visitor) override
    {
        p_visitor.visitInverter(*this);
    }
};

// ****************************************************************************
//! \brief The RunOnce decorator executes its child only once and remembers
//! the result. On subsequent ticks, it returns the cached result without
//! re-executing the child.
// ****************************************************************************
class RunOnce final: public Decorator
{
public:

    // ------------------------------------------------------------------------
    //! \brief Get the string representation of the node type.
    //! \return The string "RunOnce".
    // ------------------------------------------------------------------------
    [[nodiscard]] static constexpr char const* toString()
    {
        return "RunOnce";
    }

    // ------------------------------------------------------------------------
    //! \brief Set up the RunOnce decorator.
    //! \return Cached status if already executed, RUNNING otherwise.
    // ------------------------------------------------------------------------
    [[nodiscard]] Status onSetUp() override
    {
        if (m_executed)
        {
            return m_cached_status;
        }
        return Status::RUNNING;
    }

    // ------------------------------------------------------------------------
    //! \brief Run the RunOnce decorator.
    //! \return The child's status (cached after first completion).
    // ------------------------------------------------------------------------
    [[nodiscard]] Status onRunning() override
    {
        if (m_executed)
        {
            return m_cached_status;
        }

        Status status = m_child->tick();

        if (status != Status::RUNNING)
        {
            m_executed = true;
            m_cached_status = status;
        }

        return status;
    }

    // ------------------------------------------------------------------------
    //! \brief Reset the RunOnce decorator to allow re-execution.
    // ------------------------------------------------------------------------
    void reset() override
    {
        Decorator::reset();
        m_executed = false;
        m_cached_status = Status::INVALID;
    }

    void accept(ConstBehaviorTreeVisitor& p_visitor) const override
    {
        p_visitor.visitRunOnce(*this);
    }
    void accept(BehaviorTreeVisitor& p_visitor) override
    {
        p_visitor.visitRunOnce(*this);
    }

private:

    bool m_executed = false;
    Status m_cached_status = Status::INVALID;
};

} // namespace robotik::bt
