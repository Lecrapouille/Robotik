/**
 * @file Application.cpp
 * @brief Application base class for the viewer.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#include "Robotik/Viewer/Application/Application.hpp"

#include <future>
#include <thread>

namespace robotik::viewer::application
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
bool Application::run(size_t const p_target_fps,
                      size_t const p_target_physics_hz)
{
    if (!setup())
    {
        return false;
    }

    // Initialize rendering timing controller
    m_rendering_timing.initialize(p_target_fps,
                                  [this](size_t fps) { onFPSUpdated(fps); });

    startPhysicsThread(p_target_physics_hz);

    // Main application loop
    while (!shallBeHalted())
    {
        float delta_time = m_rendering_timing.startFrame();

        onUpdate(delta_time);
        draw();

        m_rendering_timing.endFrame();
    }

    stopPhysicsThread();
    teardown();

    return true;
}

// ----------------------------------------------------------------------------
bool Application::startPhysicsThread(size_t const p_target_physics_hz)
{
    m_physics_timing.initialize(p_target_physics_hz,
                                [this](float dt) { onPhysicUpdate(dt); });

    try
    {
        // Stop any existing physics thread first
        stopPhysicsThread();

        m_physics_running = true;
        m_physics_thread = std::thread(
            [this]()
            {
                m_physics_timing.runLoop(m_physics_running,
                                         [this](float dt)
                                         { onPhysicUpdate(dt); });
            });
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

} // namespace robotik::viewer::application