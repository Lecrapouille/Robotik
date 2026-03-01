/**
 * @file TimerQueue.hpp
 * @brief Thread-safe timer queue for timeout and delay operations.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 */

#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <map>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <vector>

namespace robotik::bt {

// ****************************************************************************
//! \brief Thread-safe timer queue for scheduling delayed operations.
//!
//! The TimerQueue runs a background thread that executes handlers after
//! specified delays. Handlers receive a boolean indicating if the timer
//! was cancelled (true) or expired normally (false).
//!
//! Usage:
//! \code
//!   bt::TimerQueue queue;
//!   auto id = queue.add(std::chrono::seconds(5), [](bool cancelled) {
//!       if (cancelled) {
//!           std::cout << "Timer was cancelled\n";
//!       } else {
//!           std::cout << "Timer expired!\n";
//!       }
//!   });
//!   // Optionally cancel before expiry:
//!   queue.cancel(id);
//! \endcode
// ****************************************************************************
class TimerQueue
{
public:

    using Clock = std::chrono::steady_clock;
    using TimePoint = Clock::time_point;
    using Duration = Clock::duration;
    using Handler = std::function<void(bool)>;

    // ------------------------------------------------------------------------
    //! \brief Constructor - starts the background worker thread.
    // ------------------------------------------------------------------------
    TimerQueue()
    {
        m_worker = std::thread([this] { run(); });
    }

    // Disable copy/move
    TimerQueue(TimerQueue const&) = delete;
    TimerQueue& operator=(TimerQueue const&) = delete;

    // ------------------------------------------------------------------------
    //! \brief Destructor - stops the worker and cancels pending timers.
    // ------------------------------------------------------------------------
    ~TimerQueue()
    {
        stop();
        m_cv.notify_all();
        if (m_worker.joinable())
        {
            m_worker.join();
        }
    }

    // ------------------------------------------------------------------------
    //! \brief Schedule a handler to be called after the specified delay.
    //! \param[in] p_delay Duration to wait before calling the handler.
    //! \param[in] p_handler Function to call, receives bool (true if
    //! cancelled).
    //! \return Unique timer ID that can be used to cancel the timer.
    // ------------------------------------------------------------------------
    template <typename Rep, typename Period, typename Callable>
    uint64_t add(std::chrono::duration<Rep, Period> p_delay,
                 Callable&& p_handler)
    {
        static_assert(std::is_invocable<Callable, bool>::value,
                      "Handler must be callable with bool argument");

        auto id = ++m_id_counter;
        TimePoint expiry = Clock::now() + p_delay;

        {
            std::lock_guard<std::mutex> lock(m_mutex);
            auto it = m_timers.emplace(
                expiry, TimerTask{id, std::forward<Callable>(p_handler)});
            m_id_map[id] = it;
        }

        m_cv.notify_one();
        return id;
    }

    // ------------------------------------------------------------------------
    //! \brief Cancel a pending timer.
    //! \param[in] p_id The timer ID returned by add().
    //! \return true if the timer was found and cancelled, false otherwise.
    // ------------------------------------------------------------------------
    bool cancel(uint64_t p_id)
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        auto map_it = m_id_map.find(p_id);
        if (map_it != m_id_map.end())
        {
            auto timer_it = map_it->second;

            // Move handler to cancel queue for notification
            m_cancel_queue.emplace_back(std::move(timer_it->second.handler));

            // Remove from multimap and hashmap
            m_timers.erase(timer_it);
            m_id_map.erase(map_it);

            return true;
        }
        return false;
    }

    // ------------------------------------------------------------------------
    //! \brief Cancel all pending timers.
    // ------------------------------------------------------------------------
    void clear()
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        for (auto& [time, task] : m_timers)
        {
            m_cancel_queue.emplace_back(std::move(task.handler));
        }
        m_timers.clear();
        m_id_map.clear();
    }

private:

    struct TimerTask
    {
        uint64_t id;
        Handler handler;
    };

    struct TimerCompare
    {
        bool operator()(TimePoint const& a, TimePoint const& b) const
        {
            return a < b;
        }
    };

    using TimerMultimap = std::multimap<TimePoint, TimerTask, TimerCompare>;

    void run()
    {
        while (m_running || !m_timers.empty())
        {
            std::unique_lock<std::mutex> lock(m_mutex);

            // Process cancellations first
            processCancellations(lock);

            if (m_timers.empty())
            {
                m_cv.wait(lock,
                          [this] { return !m_running || !m_timers.empty(); });
                continue;
            }

            auto next_time = m_timers.begin()->first;
            if (m_cv.wait_until(lock, next_time) == std::cv_status::no_timeout)
            {
                // Woken up early due to new task or cancellation
                continue;
            }

            // Execute all expired tasks
            auto now = Clock::now();
            while (!m_timers.empty() && m_timers.begin()->first <= now)
            {
                auto task = std::move(m_timers.begin()->second);
                m_timers.erase(m_timers.begin());
                m_id_map.erase(task.id);

                lock.unlock();
                try
                {
                    task.handler(false); // Normal expiry
                }
                catch (...)
                {
                }
                lock.lock();
            }
        }
    }

    void processCancellations(std::unique_lock<std::mutex>& p_lock)
    {
        for (auto&& handler : m_cancel_queue)
        {
            p_lock.unlock();
            try
            {
                handler(true); // Cancelled
            }
            catch (...)
            {
            }
            p_lock.lock();
        }
        m_cancel_queue.clear();
    }

    void stop()
    {
        m_running = false;
        clear();
    }

    std::mutex m_mutex;
    std::condition_variable_any m_cv;
    std::atomic<uint64_t> m_id_counter{0};
    std::atomic<bool> m_running{true};
    TimerMultimap m_timers;
    std::unordered_map<uint64_t, TimerMultimap::iterator> m_id_map;
    std::vector<Handler> m_cancel_queue;
    std::thread m_worker;
};

} // namespace robotik::bt
