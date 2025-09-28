/**
 * @file Application.cpp
 * @brief Application base class for the viewer.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#include "Viewer/Application.hpp"

#include <chrono>

namespace robotik
{

// *****************************************************************************
//! \brief Simple timer for application timing.
// *****************************************************************************
class AppTimer
{
    using Clock = std::chrono::steady_clock;
    using Second = std::chrono::duration<float, std::ratio<1>>;

public:

    AppTimer()
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
Application::Application(Path& p_path,
                         size_t window_width,
                         size_t window_height,
                         const std::string& window_title)
    : m_viewer(p_path, window_width, window_height, window_title)
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
bool Application::isHalting() const
{
    return m_viewer.isHalting();
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

    AppTimer timer;
    float time_since_last_update = 0.0f;
    const float time_per_frame = 1.0f / float(p_target_fps);

    // Main application loop - Proper 60 FPS timing
    while (!isHalting())
    {
        // Process input events every frame
        m_viewer.processInput([this](int key, int action)
                              { handleInput(key, action); });

        // Update application logic at target FPS
        time_since_last_update += timer.restart();
        if (time_since_last_update >= time_per_frame)
        {
            time_since_last_update -= time_per_frame;
            onUpdate(time_per_frame);
        }

        // Draw the application every frame
        onDraw();
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
                std::chrono::microseconds(1000000 / p_target_physics_hz);
            const auto physics_dt_seconds = 1.0f / float(p_target_physics_hz);
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