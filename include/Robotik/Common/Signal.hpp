// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file Signal.hpp
/// A way for one part of the program to tell others that something happened,
/// without knowing who listens (the "signal and slot" idea of Qt).
///
/// The part that knows (a skill scheduler, a camera) owns a @ref Signal and
/// calls it when the event happens. Anyone interested connects a function
/// (a "slot") to it. Neither side includes the other: a simulator panel can
/// react to a finished skill without the scheduler knowing panels exist.
///
/// @code
/// robotik::Signal<std::string const&, bool> skill_done;
/// auto const id = skill_done.connect([](std::string const& p_name, bool p_ok) {
///     std::cout << p_name << (p_ok ? " succeeded\n" : " failed\n");
/// });
/// skill_done("Pick", true);   // prints "Pick succeeded"
/// skill_done.disconnect(id);
/// @endcode
#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <utility>
#include <vector>

namespace robotik
{

// ****************************************************************************
/// Calls every connected function when the event happens, in connection
/// order.
///
/// A function may connect or disconnect slots while the signal is being
/// emitted: a slot disconnected during an emission is not called afterwards,
/// a slot connected during an emission is first called at the next one.
///
/// A signal cannot be copied nor moved: connections refer to it.
///
/// @tparam Args What the event tells its listeners.
// ****************************************************************************
template <typename... Args>
class Signal
{
public:

    /// Function called when the signal is emitted.
    using Slot = std::function<void(Args...)>;
    /// Handle of a connection, to disconnect it.
    using Id = std::uint32_t;

    Signal() = default;
    Signal(Signal const&) = delete;
    Signal& operator=(Signal const&) = delete;
    Signal(Signal&&) = delete;
    Signal& operator=(Signal&&) = delete;
    ~Signal() = default;

    // -------------------------------------------------------------------------
    /// Connects a function.
    /// @param p_slot Function, lambda or bound method.
    /// @return Handle for @ref disconnect.
    // -------------------------------------------------------------------------
    Id connect(Slot p_slot)
    {
        Id const id = m_next++;
        m_slots.push_back({ id, std::move(p_slot) });
        return id;
    }

    // -------------------------------------------------------------------------
    /// Disconnects a function. Unknown handles are ignored.
    /// @param p_id Handle returned by @ref connect.
    // -------------------------------------------------------------------------
    void disconnect(Id p_id)
    {
        for (Entry& entry : m_slots)
        {
            if (entry.id == p_id)
            {
                entry.slot = nullptr;
            }
        }
        compact();
    }

    // -------------------------------------------------------------------------
    /// Disconnects every function.
    // -------------------------------------------------------------------------
    void disconnectAll()
    {
        for (Entry& entry : m_slots)
        {
            entry.slot = nullptr;
        }
        compact();
    }

    // -------------------------------------------------------------------------
    /// Calls every connected function with the event.
    /// @param p_args What the event tells.
    // -------------------------------------------------------------------------
    void emit(Args... p_args)
    {
        ++m_emitting;
        std::size_t const count = m_slots.size(); // not the slots added now
        for (std::size_t i = 0; i < count; ++i)
        {
            if (m_slots[i].slot)
            {
                m_slots[i].slot(p_args...);
            }
        }
        --m_emitting;
        compact();
    }

    /// Same as @ref emit.
    void operator()(Args... p_args)
    {
        emit(p_args...);
    }

    // -------------------------------------------------------------------------
    /// @return The number of connected functions.
    // -------------------------------------------------------------------------
    [[nodiscard]] std::size_t size() const
    {
        std::size_t count = 0;
        for (Entry const& entry : m_slots)
        {
            count += entry.slot ? 1U : 0U;
        }
        return count;
    }

private:

    /// One connected function.
    struct Entry
    {
        Id id;     ///< Handle given to the caller.
        Slot slot; ///< The function; empty once disconnected.
    };

    /// Removes the disconnected functions, unless an emission is going on.
    void compact()
    {
        if (m_emitting != 0)
        {
            return;
        }
        std::erase_if(m_slots, [](Entry const& p_entry) { return !p_entry.slot; });
    }

    /// Connected functions, in connection order.
    std::vector<Entry> m_slots;
    /// Next handle.
    Id m_next = 0;
    /// Depth of the emissions in progress.
    int m_emitting = 0;
};

// ****************************************************************************
/// Connection that ends by itself when it goes out of scope.
///
/// Use it when the listener dies before the signal: a panel that listens to
/// a simulation while it is open.
///
/// @code
/// class Panel
/// {
/// public:
///     explicit Panel(robotik::Signal<int>& p_signal)
///         : m_link(p_signal, [this](int p_value) { m_last = p_value; })
///     {
///     }
/// private:
///     int m_last = 0;
///     robotik::ScopedConnection<int> m_link; // disconnects in ~Panel
/// };
/// @endcode
// ****************************************************************************
template <typename... Args>
class ScopedConnection
{
public:

    ScopedConnection() = default;

    // -------------------------------------------------------------------------
    /// Connects @p_slot to @p_signal until this object is destroyed.
    /// @param p_signal Signal to listen to; it must outlive this object.
    /// @param p_slot Function to call.
    // -------------------------------------------------------------------------
    ScopedConnection(Signal<Args...>& p_signal, typename Signal<Args...>::Slot p_slot)
        : m_signal(&p_signal), m_id(p_signal.connect(std::move(p_slot)))
    {
    }

    ScopedConnection(ScopedConnection const&) = delete;
    ScopedConnection& operator=(ScopedConnection const&) = delete;

    /// Takes the connection of @p_other, which becomes empty.
    ScopedConnection(ScopedConnection&& p_other) noexcept
        : m_signal(std::exchange(p_other.m_signal, nullptr)), m_id(p_other.m_id)
    {
    }

    /// Ends the current connection, then takes the one of @p_other.
    ScopedConnection& operator=(ScopedConnection&& p_other) noexcept
    {
        if (this != &p_other)
        {
            disconnect();
            m_signal = std::exchange(p_other.m_signal, nullptr);
            m_id = p_other.m_id;
        }
        return *this;
    }

    /// Ends the connection.
    ~ScopedConnection()
    {
        disconnect();
    }

    // -------------------------------------------------------------------------
    /// Ends the connection now; nothing happens if it already ended.
    // -------------------------------------------------------------------------
    void disconnect()
    {
        if (m_signal != nullptr)
        {
            m_signal->disconnect(m_id);
            m_signal = nullptr;
        }
    }

private:

    /// Signal listened to, or null.
    Signal<Args...>* m_signal = nullptr;
    /// Handle of the connection.
    typename Signal<Args...>::Id m_id = 0;
};

} // namespace robotik
