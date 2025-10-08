/**
 * @file Application.hpp
 * @brief Application base class for the viewer.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#pragma once

#include <atomic>
#include <string>
#include <thread>

namespace robotik::viewer
{

// Forward declaration
class OpenGLWindow;

// ****************************************************************************
//! \brief Base application class providing a common interface for all
//! applications.
// ****************************************************************************
class Application
{
public:

    // ----------------------------------------------------------------------------
    //! \brief Destructor.
    // ----------------------------------------------------------------------------
    virtual ~Application();

    // ----------------------------------------------------------------------------
    //! \brief Start the application, start the physics thread and run the main
    //! loop.
    //! \param p_target_fps Target rendering frame rate.
    //! \param p_target_physics_hz Target physics update rate.
    //! \return true if the application should close, false otherwise.
    // ----------------------------------------------------------------------------
    bool run(size_t const p_target_fps, size_t const p_target_physics_hz);

    // ----------------------------------------------------------------------------
    //! \brief Get the last error message from the physics thread.
    //! \return The error message, or empty string if no error.
    // ----------------------------------------------------------------------------
    std::string const& error() const;

    // ----------------------------------------------------------------------------
    //! \brief Set the title of the application.
    //! \param p_title The title of the application.
    // ----------------------------------------------------------------------------
    virtual void setTitle(std::string const& p_title) = 0;

private:

    // ----------------------------------------------------------------------------
    //! \brief Check if the application should close.
    //! \return true if the application should close, false otherwise.
    // ----------------------------------------------------------------------------
    virtual bool isHalting() const;

    // ----------------------------------------------------------------------------
    //! \brief Initialize the application. Called once at startup.
    //! \return true if initialization was successful, false otherwise.
    // ----------------------------------------------------------------------------
    virtual bool onSetup() = 0;

    // ----------------------------------------------------------------------------
    //! \brief Cleanup the application. Called once at shutdown.
    // ----------------------------------------------------------------------------
    virtual void onCleanup() = 0;

    // ----------------------------------------------------------------------------
    //! \brief Render the application. Called every frame.
    // ----------------------------------------------------------------------------
    virtual void onDraw() = 0;

    // ----------------------------------------------------------------------------
    //! \brief Update the application logic. Called every frame.
    //! \param p_dt Delta time in seconds since last frame.
    // ----------------------------------------------------------------------------
    virtual void onUpdate(float const p_dt) = 0;

    // ----------------------------------------------------------------------------
    //! \brief Update physics simulation. Called at fixed time intervals.
    //! \param p_dt Fixed delta time for physics simulation.
    //! \note This method is called internally by the physics thread.
    // ----------------------------------------------------------------------------
    virtual void onPhysicUpdate(float const p_dt) = 0;

    // ----------------------------------------------------------------------------
    //! \brief Update the FPS.
    //! \param p_fps Current FPS value.
    // ----------------------------------------------------------------------------
    virtual void onFPSUpdated(size_t const p_fps) = 0;

private:

    // ----------------------------------------------------------------------------
    //! \brief Start the physics thread.
    //! \param p_target_physics_hz Target physics update rate.
    //! \return true if the physics thread was started successfully, false
    //! otherwise. Call error() to get the error message.
    // ----------------------------------------------------------------------------
    bool startPhysicsThread(size_t const p_target_physics_hz);

    // ----------------------------------------------------------------------------
    //! \brief Stop the physics thread.
    //! \return true if the physics thread was stopped successfully, false
    //! otherwise. Call error() to get the error message.
    // ----------------------------------------------------------------------------
    bool stopPhysicsThread();

    // ----------------------------------------------------------------------------
    //! \brief Physics thread main loop.
    //! \param p_target_physics_hz Target physics update rate.
    // ----------------------------------------------------------------------------
    void physicsThreadLoop(size_t const p_target_physics_hz);

    // ----------------------------------------------------------------------------
    //! \brief Update the FPS calculation based on frame count.
    // ----------------------------------------------------------------------------
    void updateFPS();

    // ----------------------------------------------------------------------------
    //! \brief Limit the framerate.
    // ----------------------------------------------------------------------------
    void limitFramerate(float const p_target_frame_time) const;

protected:

    std::string m_error;

private:

    std::chrono::high_resolution_clock::time_point m_last_frame_time;
    std::thread m_physics_thread{};
    std::atomic<bool> m_physics_running{ false };
    size_t m_frame_count;
};

} // namespace robotik::viewer