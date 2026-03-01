/**
 * @file Temporal.hpp
 * @brief Temporal decorator nodes: Timeout, Delay, Cooldown.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 */

#pragma once

#include "Robotik/Core/BehaviorTree/Core/Decorator.hpp"

#include <chrono>

namespace robotik::bt {

// ****************************************************************************
//! \brief The Timeout decorator runs its child with a time limit.
//! If the child does not complete within the specified time, it returns
//! FAILURE and halts the child. This is useful for preventing long-running
//! actions from blocking the tree.
//! The timeout duration can be read from the blackboard via port remapping.
// ****************************************************************************
class Timeout final: public Decorator
{
public:

    using Clock = std::chrono::steady_clock;
    using Duration = std::chrono::milliseconds;

    // ------------------------------------------------------------------------
    //! \brief Get the string representation of the node type.
    //! \return The string "Timeout".
    // ------------------------------------------------------------------------
    [[nodiscard]] static constexpr char const* toString()
    {
        return "Timeout";
    }

    // ------------------------------------------------------------------------
    //! \brief Constructor taking a timeout duration.
    //! \param[in] p_milliseconds The timeout duration in milliseconds.
    // ------------------------------------------------------------------------
    explicit Timeout(size_t p_milliseconds = 1000)
        : m_default_timeout(p_milliseconds)
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
        ports.addInput<size_t>("milliseconds");
        return ports;
    }

    // ------------------------------------------------------------------------
    //! \brief Set up the timeout - records the start time.
    //! Reads timeout from blackboard if configured, otherwise uses default.
    //! \return RUNNING to continue with onRunning.
    // ------------------------------------------------------------------------
    [[nodiscard]] Status onSetUp() override
    {
        // Try to read timeout from blackboard (try int first, then size_t)
        if (auto ms = getInput<int>("milliseconds"); ms && *ms >= 0)
        {
            m_timeout = Duration(static_cast<size_t>(*ms));
        }
        else if (auto ms2 = getInput<size_t>("milliseconds"); ms2)
        {
            m_timeout = Duration(*ms2);
        }
        else
        {
            m_timeout = Duration(m_default_timeout);
        }
        m_start_time = Clock::now();
        return Status::RUNNING;
    }

    // ------------------------------------------------------------------------
    //! \brief Run the timeout decorator.
    //! \return FAILURE if timeout expired, child's status otherwise.
    // ------------------------------------------------------------------------
    [[nodiscard]] Status onRunning() override
    {
        auto elapsed =
            std::chrono::duration_cast<Duration>(Clock::now() - m_start_time);

        if (elapsed >= m_timeout)
        {
            // Timeout expired, halt child if running
            if (m_child->status() == Status::RUNNING)
            {
                m_child->halt();
            }
            return Status::FAILURE;
        }

        // Still have time, tick child
        return m_child->tick();
    }

    // ------------------------------------------------------------------------
    //! \brief Get the timeout duration in milliseconds.
    //! \return The timeout duration.
    // ------------------------------------------------------------------------
    [[nodiscard]] size_t getMilliseconds() const
    {
        return static_cast<size_t>(m_timeout.count());
    }

    void accept(ConstBehaviorTreeVisitor& p_visitor) const override
    {
        p_visitor.visitTimeout(*this);
    }
    void accept(BehaviorTreeVisitor& p_visitor) override
    {
        p_visitor.visitTimeout(*this);
    }

private:

    size_t m_default_timeout;
    Duration m_timeout;
    Clock::time_point m_start_time;
};

// ****************************************************************************
//! \brief The Delay decorator waits for a specified duration before
//! starting to tick its child. Once the delay has passed, it ticks the child
//! and returns its status.
// ****************************************************************************
class Delay final: public Decorator
{
public:

    using Clock = std::chrono::steady_clock;
    using Duration = std::chrono::milliseconds;

    // ------------------------------------------------------------------------
    //! \brief Get the string representation of the node type.
    //! \return The string "⏳ Delay".
    // ------------------------------------------------------------------------
    [[nodiscard]] static constexpr char const* toString()
    {
        return "⏳ Delay";
    }

    // ------------------------------------------------------------------------
    //! \brief Constructor taking a delay duration.
    //! \param[in] p_milliseconds The delay duration in milliseconds.
    // ------------------------------------------------------------------------
    explicit Delay(size_t p_milliseconds = 1000)
        : m_delay(Duration(p_milliseconds))
    {
    }

    // ------------------------------------------------------------------------
    //! \brief Set up the delay - records the start time.
    //! \return RUNNING to continue with onRunning.
    // ------------------------------------------------------------------------
    [[nodiscard]] Status onSetUp() override
    {
        m_start_time = Clock::now();
        m_delay_passed = false;
        return Status::RUNNING;
    }

    // ------------------------------------------------------------------------
    //! \brief Run the delay decorator.
    //! \return RUNNING during delay, child's status after delay.
    // ------------------------------------------------------------------------
    [[nodiscard]] Status onRunning() override
    {
        if (!m_delay_passed)
        {
            auto elapsed = std::chrono::duration_cast<Duration>(Clock::now() -
                                                                m_start_time);

            if (elapsed < m_delay)
            {
                return Status::RUNNING;
            }
            m_delay_passed = true;
        }

        // Delay passed, tick child
        return m_child->tick();
    }

    // ------------------------------------------------------------------------
    //! \brief Get the delay duration in milliseconds.
    //! \return The delay duration.
    // ------------------------------------------------------------------------
    [[nodiscard]] size_t getMilliseconds() const
    {
        return static_cast<size_t>(m_delay.count());
    }

    void accept(ConstBehaviorTreeVisitor& p_visitor) const override
    {
        p_visitor.visitDelay(*this);
    }
    void accept(BehaviorTreeVisitor& p_visitor) override
    {
        p_visitor.visitDelay(*this);
    }

private:

    Duration m_delay;
    Clock::time_point m_start_time;
    bool m_delay_passed = false;
};

// ****************************************************************************
//! \brief The Cooldown decorator prevents the child from being executed
//! more frequently than the specified interval. After the child completes
//! (SUCCESS or FAILURE), subsequent ticks return FAILURE until the cooldown
//! period has elapsed.
// ****************************************************************************
class Cooldown final: public Decorator
{
public:

    using Clock = std::chrono::steady_clock;
    using Duration = std::chrono::milliseconds;

    // ------------------------------------------------------------------------
    //! \brief Get the string representation of the node type.
    //! \return The string "❄️ Cooldown".
    // ------------------------------------------------------------------------
    [[nodiscard]] static constexpr char const* toString()
    {
        return "❄️ Cooldown";
    }

    // ------------------------------------------------------------------------
    //! \brief Constructor taking a cooldown duration.
    //! \param[in] p_milliseconds The cooldown duration in milliseconds.
    // ------------------------------------------------------------------------
    explicit Cooldown(size_t p_milliseconds = 1000)
        : m_cooldown(Duration(p_milliseconds))
    {
    }

    // ------------------------------------------------------------------------
    //! \brief Set up the cooldown decorator.
    //! \return RUNNING or FAILURE depending on cooldown state.
    // ------------------------------------------------------------------------
    [[nodiscard]] Status onSetUp() override
    {
        // Check if we're still in cooldown
        if (m_in_cooldown)
        {
            auto elapsed = std::chrono::duration_cast<Duration>(
                Clock::now() - m_cooldown_start);

            if (elapsed < m_cooldown)
            {
                return Status::FAILURE; // Still in cooldown
            }
            m_in_cooldown = false;
        }
        return Status::RUNNING;
    }

    // ------------------------------------------------------------------------
    //! \brief Run the cooldown decorator.
    //! \return The child's status.
    // ------------------------------------------------------------------------
    [[nodiscard]] Status onRunning() override
    {
        Status status = m_child->tick();

        if (status != Status::RUNNING)
        {
            // Child finished, start cooldown
            m_cooldown_start = Clock::now();
            m_in_cooldown = true;
        }

        return status;
    }

    // ------------------------------------------------------------------------
    //! \brief Get the cooldown duration in milliseconds.
    //! \return The cooldown duration.
    // ------------------------------------------------------------------------
    [[nodiscard]] size_t getMilliseconds() const
    {
        return static_cast<size_t>(m_cooldown.count());
    }

    void accept(ConstBehaviorTreeVisitor& p_visitor) const override
    {
        p_visitor.visitCooldown(*this);
    }
    void accept(BehaviorTreeVisitor& p_visitor) override
    {
        p_visitor.visitCooldown(*this);
    }

private:

    Duration m_cooldown;
    Clock::time_point m_cooldown_start;
    bool m_in_cooldown = false;
};

} // namespace robotik::bt
