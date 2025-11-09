/**
 * @file Signal.hpp
 * @brief Signal/Slot system for event handling.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#pragma once

#include <cstddef>
#include <functional>
#include <map>

namespace robotik
{

// ****************************************************************************
//! \brief Signal/Slot system for event handling.
//!
//! This class provides a signal/slot mechanism for connecting callbacks
//! to events. Slots can be connected and disconnected, and signals can
//! be emitted to call all connected slots.
//!
//! \tparam Args Parameter types for the signal.
// ****************************************************************************
template <typename... Args>
class Signal
{
public:

    using SlotType = std::function<void(Args...)>;
    using ConnectionId = size_t;

    // ------------------------------------------------------------------------
    //! \brief Connect a slot (function, lambda, method).
    //! \param slot The slot to connect.
    //! \return Connection ID for later disconnection.
    // ------------------------------------------------------------------------
    ConnectionId connect(SlotType slot)
    {
        ConnectionId id = next_id_++;
        slots_[id] = std::move(slot);
        return id;
    }

    // ------------------------------------------------------------------------
    //! \brief Disconnect a slot by its connection ID.
    //! \param id Connection ID to disconnect.
    // ------------------------------------------------------------------------
    void disconnect(ConnectionId id)
    {
        slots_.erase(id);
    }

    // ------------------------------------------------------------------------
    //! \brief Disconnect all slots.
    // ------------------------------------------------------------------------
    void disconnectAll()
    {
        slots_.clear();
    }

    // ------------------------------------------------------------------------
    //! \brief Emit the signal, calling all connected slots.
    //! \param args Arguments to pass to the slots.
    // ------------------------------------------------------------------------
    void emit(Args... args)
    {
        // Copy to avoid issues if a slot disconnects during emission
        auto slots_copy = slots_;
        for (const auto& [id, slot] : slots_copy)
        {
            if (slots_.count(id)) // Check if still connected
            {
                slot(args...);
            }
        }
    }

    // ------------------------------------------------------------------------
    //! \brief Operator() for more natural syntax.
    //! \param args Arguments to pass to the slots.
    // ------------------------------------------------------------------------
    void operator()(Args... args)
    {
        emit(args...);
    }

    // ------------------------------------------------------------------------
    //! \brief Get the number of connected slots.
    //! \return Number of connections.
    // ------------------------------------------------------------------------
    size_t connectionCount() const
    {
        return slots_.size();
    }

private:

    //! \brief Map of connection IDs to slots.
    std::map<ConnectionId, SlotType> slots_;
    //! \brief Next connection ID to assign.
    ConnectionId next_id_ = 0;
};

// ****************************************************************************
//! \brief RAII wrapper for automatic connection management.
//!
//! This class automatically disconnects a signal connection when it goes
//! out of scope.
//!
//! \tparam Args Parameter types for the signal.
// ****************************************************************************
template <typename... Args>
class ScopedConnection
{
public:

    // ------------------------------------------------------------------------
    //! \brief Constructor.
    //! \param signal Pointer to the signal.
    //! \param id Connection ID.
    // ------------------------------------------------------------------------
    ScopedConnection(Signal<Args...>* signal,
                     typename Signal<Args...>::ConnectionId id)
        : signal_(signal), id_(id)
    {
    }

    // ------------------------------------------------------------------------
    //! \brief Destructor - automatically disconnects.
    // ------------------------------------------------------------------------
    ~ScopedConnection()
    {
        if (signal_)
        {
            signal_->disconnect(id_);
        }
    }

    // Delete copy operations
    ScopedConnection(const ScopedConnection&) = delete;
    ScopedConnection& operator=(const ScopedConnection&) = delete;

    // ------------------------------------------------------------------------
    //! \brief Move constructor.
    // ------------------------------------------------------------------------
    ScopedConnection(ScopedConnection&& other) noexcept
        : signal_(other.signal_), id_(other.id_)
    {
        other.signal_ = nullptr;
    }

    // ------------------------------------------------------------------------
    //! \brief Move assignment operator.
    // ------------------------------------------------------------------------
    ScopedConnection& operator=(ScopedConnection&& other) noexcept
    {
        if (this != &other)
        {
            // Disconnect current connection
            if (signal_)
            {
                signal_->disconnect(id_);
            }
            // Move from other
            signal_ = other.signal_;
            id_ = other.id_;
            other.signal_ = nullptr;
        }
        return *this;
    }

private:

    //! \brief Pointer to the signal.
    Signal<Args...>* signal_;
    //! \brief Connection ID.
    typename Signal<Args...>::ConnectionId id_;
};

} // namespace robotik
