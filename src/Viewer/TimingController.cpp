#include "Robotik/Viewer/TimingController.hpp"

#include <algorithm>
#include <thread>

namespace robotik::viewer
{

// ----------------------------------------------------------------------------
void TimingController::initialize(size_t p_target_rate,
                                  RateCallback const& p_rate_callback)
{
    m_target_frame_time = 1.0f / static_cast<float>(p_target_rate);
    m_rate_callback = p_rate_callback;
    m_frame_count = 0;
    m_last_frame_count = 0;
}

// ----------------------------------------------------------------------------
float TimingController::startFrame()
{
    auto current_time = std::chrono::high_resolution_clock::now();

    // First frame initialization
    if (m_frame_count == 0)
    {
        m_last_frame_time = current_time;
        m_last_rate_time = current_time;
        m_frame_count++;
        return 0.0f;
    }

    // Calculate delta time
    float delta_time =
        std::chrono::duration<float>(current_time - m_last_frame_time).count();

    m_last_frame_time = current_time;
    m_frame_count++;

    return delta_time;
}

// ----------------------------------------------------------------------------
void TimingController::endFrame()
{
    updateRate();
    limitFramerate();
}

// ----------------------------------------------------------------------------
void TimingController::runLoop(std::atomic<bool> const& p_running,
                               UpdateCallback const& p_update_callback) const
{
    using Clock = std::chrono::high_resolution_clock;

    const auto target_dt = std::chrono::duration<double>(m_target_frame_time);
    auto next_update = Clock::now();
    auto last_update = Clock::now();

    while (p_running.load())
    {
        // Calculate actual delta time
        auto current_time = Clock::now();
        auto actual_dt =
            std::chrono::duration<float>(current_time - last_update).count();
        last_update = current_time;

        // Use actual delta time if reasonable, otherwise use fixed timestep
        float dt_to_use = (actual_dt > 0.0f && actual_dt < 0.1f)
                              ? actual_dt
                              : m_target_frame_time;

        // Clamp delta time to prevent issues
        dt_to_use = std::clamp(dt_to_use, 0.001f, 0.1f);

        // Call update callback
        if (p_update_callback)
        {
            p_update_callback(dt_to_use);
        }

        // Calculate next update time
        next_update += std::chrono::duration_cast<Clock::duration>(target_dt);

        // Sleep until next update
        auto now = Clock::now();
        if (next_update > now)
        {
            std::this_thread::sleep_until(next_update);
        }
        else
        {
            // We're behind schedule, catch up
            next_update =
                now + std::chrono::duration_cast<Clock::duration>(target_dt);
        }
    }
}

// ----------------------------------------------------------------------------
float TimingController::currentRate() const
{
    return m_current_rate;
}

// ----------------------------------------------------------------------------
size_t TimingController::frameCount() const
{
    return m_frame_count;
}

// ----------------------------------------------------------------------------
void TimingController::updateRate()
{
    auto current_time = std::chrono::high_resolution_clock::now();
    auto time_elapsed =
        std::chrono::duration<float>(current_time - m_last_rate_time).count();

    // Update rate every second
    if (time_elapsed >= 1.0f)
    {
        size_t frames_this_second = m_frame_count - m_last_frame_count;
        m_current_rate = static_cast<float>(frames_this_second) / time_elapsed;

        // Call the callback if set
        if (m_rate_callback)
        {
            m_rate_callback(static_cast<size_t>(m_current_rate));
        }

        // Reset counters
        m_last_rate_time = current_time;
        m_last_frame_count = m_frame_count;
    }
}

// ----------------------------------------------------------------------------
void TimingController::limitFramerate() const
{
    auto current_time = std::chrono::high_resolution_clock::now();
    float frame_time =
        std::chrono::duration<float>(current_time - m_last_frame_time).count();

    if (frame_time < m_target_frame_time)
    {
        float sleep_time = m_target_frame_time - frame_time;
        std::this_thread::sleep_for(std::chrono::duration<float>(sleep_time));
    }
}

} // namespace robotik::viewer
