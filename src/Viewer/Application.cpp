/**
 * @file Application.cpp
 * @brief Application base class for the viewer.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#include "Robotik/Viewer/Application.hpp"

#include <algorithm>
#include <chrono>
#include <future>
#include <thread>

namespace robotik::viewer
{

// ----------------------------------------------------------------------------
Application::~Application()
{
    stopPhysicsThread();
}

// ----------------------------------------------------------------------------
std::string const& Application::error() const
{
    return m_error;
}

// ----------------------------------------------------------------------------
void Application::updateFPS()
{
    // Calculate FPS based on frame count and time elapsed
    // This gives a more accurate average FPS over the last second
    static auto last_fps_time = std::chrono::high_resolution_clock::now();
    static size_t last_frame_count = 0;

    auto current_time = std::chrono::high_resolution_clock::now();
    auto time_elapsed =
        std::chrono::duration<float>(current_time - last_fps_time).count();

    // Update FPS every second
    if (time_elapsed >= 1.0f)
    {
        size_t frames_this_second = m_frame_count - last_frame_count;
        float current_fps =
            static_cast<float>(frames_this_second) / time_elapsed;

        onFPSUpdated(static_cast<size_t>(current_fps));

        // Reset counters
        last_fps_time = current_time;
        last_frame_count = m_frame_count;
    }
    m_frame_count++;
}

// ----------------------------------------------------------------------------
void Application::limitFramerate(float const p_target_frame_time) const
{
    auto currentTime = std::chrono::high_resolution_clock::now();
    float frameTime =
        std::chrono::duration<float>(currentTime - m_last_frame_time).count();

    if (frameTime < p_target_frame_time)
    {
        float sleepTime = p_target_frame_time - frameTime;
        std::this_thread::sleep_for(std::chrono::duration<float>(sleepTime));
    }
}

// ----------------------------------------------------------------------------
bool Application::run(size_t const p_target_fps,
                      size_t const p_target_physics_hz)
{
    // User setup callback
    if (!onSetup())
    {
        return false;
    }

    startPhysicsThread(p_target_physics_hz);

    // Main application loop
    float target_frame_time = 1.0f / static_cast<float>(p_target_fps);
    m_last_frame_time = std::chrono::high_resolution_clock::now();
    while (!isHalting())
    {
        // Calculate the delta time for the display
        auto current_time = std::chrono::high_resolution_clock::now();
        float delta_time =
            std::chrono::duration<float>(current_time - m_last_frame_time)
                .count();
        m_last_frame_time = current_time;

        // User setup callbacks
        onUpdate(delta_time);
        onDraw();

        // Update the FPS and limit the framerate
        updateFPS();
        limitFramerate(target_frame_time);
    }

    stopPhysicsThread();

    // User setup callback
    onCleanup();

    return true;
}

// ----------------------------------------------------------------------------
bool Application::startPhysicsThread(size_t const p_target_physics_hz)
{
    try
    {
        // Stop any existing physics thread first
        stopPhysicsThread();

        m_physics_running = true;
        m_physics_thread = std::thread(
            &Application::physicsThreadLoop, this, p_target_physics_hz);
    }
    catch (const std::exception& e)
    {
        // Store error for later retrieval
        m_error = "Physics thread exception: " + std::string(e.what());
        m_physics_running = false;
    }
    return m_physics_running;
}

// ----------------------------------------------------------------------------
void Application::physicsThreadLoop(size_t const p_target_physics_hz)
{
    try
    {
        // Use high resolution clock for better precision
        using Clock = std::chrono::high_resolution_clock;
        const auto physics_dt = std::chrono::duration<double>(
            1.0 / static_cast<double>(p_target_physics_hz));
        const float physics_dt_seconds =
            1.0f / static_cast<float>(p_target_physics_hz);

        auto next_update = Clock::now();
        auto last_update = Clock::now();

        while (m_physics_running)
        {
            // Calculate actual delta time for this physics step
            auto current_time = Clock::now();
            auto actual_dt =
                std::chrono::duration<float>(current_time - last_update)
                    .count();
            last_update = current_time;

            // Use actual delta time if reasonable, otherwise use fixed
            // timestep
            float dt_to_use = (actual_dt > 0.0f && actual_dt < 0.1f)
                                  ? actual_dt
                                  : physics_dt_seconds;

            // Clamp delta time to prevent physics explosions
            dt_to_use = std::clamp(dt_to_use, 0.001f, 0.1f);

            onPhysicUpdate(dt_to_use);

            // Calculate next update time with better precision
            next_update +=
                std::chrono::duration_cast<Clock::duration>(physics_dt);

            // Sleep until next update
            auto now = Clock::now();
            if (next_update > now)
            {
                std::this_thread::sleep_until(next_update);
            }
            else
            {
                // We're behind schedule, don't sleep and catch up
                next_update = now + std::chrono::duration_cast<Clock::duration>(
                                        physics_dt);
            }
        }
    }
    catch (const std::exception& e)
    {
        // Store error for later retrieval
        m_error = "Physics thread exception: " + std::string(e.what());
        m_physics_running = false;
    }
}

// ----------------------------------------------------------------------------
bool Application::stopPhysicsThread()
{
    try
    {
        if (m_physics_running)
        {
            m_physics_running = false;

            if (m_physics_thread.joinable())
            {
                // Give the thread a reasonable time to finish gracefully
                auto future = std::async(std::launch::async,
                                         [this]() { m_physics_thread.join(); });

                // Wait for join with timeout
                if (future.wait_for(std::chrono::milliseconds(500)) ==
                    std::future_status::timeout)
                {
                    // Thread didn't join in time, detach it (not ideal but
                    // prevents hang)
                    m_physics_thread.detach();
                }
            }
        }
        return true;
    }
    catch (std::exception const& e)
    {
        // Handle any other exceptions
        m_error = "Physics thread exception: " + std::string(e.what());
        m_physics_running = false;
        return false;
    }
}

} // namespace robotik::viewer