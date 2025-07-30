#include "Application.hpp"

#include <chrono>
#include <iostream>
#include <thread>

namespace robotik
{

// *****************************************************************************
//! \brief
// *****************************************************************************
class Timer
{
    using Clock = std::chrono::steady_clock;
    using Second = std::chrono::duration<float, std::ratio<1>>;

public:

    Timer()
    {
        m_begin = Clock::now();
    }

    float restart()
    {
        float res = elapsed();
        m_begin = Clock::now();
        return res;
    }

    float elapsed() const
    {
        return std::chrono::duration_cast<Second>(Clock::now() - m_begin)
            .count();
    }

private:

    std::chrono::time_point<Clock> m_begin;
};

// ----------------------------------------------------------------------------
Application::Application(size_t window_width,
                         size_t window_height,
                         const std::string& window_title)
    : m_viewer(window_width, window_height, window_title)
{
}

// ----------------------------------------------------------------------------
Application::~Application()
{
    stopPhysicsThread();
}

// ----------------------------------------------------------------------------
bool Application::isKeyPressed(int p_key) const
{
    return m_viewer.isKeyPressed(p_key);
}

// ----------------------------------------------------------------------------
bool Application::shouldClose() const
{
    return m_viewer.shouldClose();
}

// ----------------------------------------------------------------------------
bool Application::run(size_t const p_target_fps,
                      size_t const p_target_physics_hz)
{
    // Initialize the OpenGL viewer
    if (!m_viewer.initialize())
    {
        return false;
    }

    // User setup callback
    if (!onSetup())
    {
        return false;
    }

    // Start physics thread
    startPhysicsThread(p_target_physics_hz);

    Timer timer;
    float time_since_last_update = 0.0f;
    const float time_per_frame = 1.0f / float(p_target_fps);

    // Main application loop
    while (!shouldClose())
    {
        // Process events at fixed time steps
        time_since_last_update += timer.restart();
        while (time_since_last_update > time_per_frame)
        {
            time_since_last_update -= time_per_frame;

            // Process input events
            m_viewer.processInput([this](int key, int action)
                                  { handleInput(key, action); });

            // Update application logic
            onUpdate(time_per_frame);

            // Draw the application
            onDraw();
        }
    }

    // Stop physics thread before exiting
    stopPhysicsThread();

    return true;
}

// ----------------------------------------------------------------------------
void Application::startPhysicsThread(size_t const p_target_physics_hz)
{
    m_physics_running = true;
    m_physics_thread = std::thread(
        [this, p_target_physics_hz]()
        {
            const auto physics_dt =
                std::chrono::microseconds(p_target_physics_hz);
            const auto physics_dt_seconds =
                float(physics_dt.count()) / 1000000.0f;
            auto next_update = std::chrono::steady_clock::now();

            while (m_physics_running)
            {
                // Call physics update with fixed timestep
                onPhysicUpdate(physics_dt_seconds);

                // Sleep until next update
                next_update += physics_dt;
                std::this_thread::sleep_until(next_update);
            }
        });
}

// ----------------------------------------------------------------------------
void Application::stopPhysicsThread()
{
    m_physics_running = false;
    if (m_physics_thread.joinable())
    {
        m_physics_thread.join();
    }
}

} // namespace robotik