/**
 * @file Application.hpp
 * @brief Application base class for the viewer.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#pragma once

#include "Robotik/Viewer/Application/TimingController.hpp"

#include <atomic>
#include <string>
#include <thread>

namespace robotik::viewer::application
{

// Forward declaration
class OpenGLWindow;

// ****************************************************************************
//! \brief Base application class providing a common interface for all
//! applications using rendering, physics and drawing GUI.
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
    //! infinite loop. This method is blocking and will return when the
    //! application halted (e.g. by user request or error).
    //! \param p_target_fps Target rendering frame rate.
    //! \param p_target_physics_hz Target physics update rate.
    //! \return true if the application closed successfully, false otherwise.
    // ----------------------------------------------------------------------------
    bool run(size_t const p_target_fps, size_t const p_target_physics_hz);

    // ----------------------------------------------------------------------------
    //! \brief Get the last error message when run() returned false.
    //! \return The error message, or empty string if the application closed
    //! successfully.
    // ----------------------------------------------------------------------------
    std::string const& error() const;

    // ----------------------------------------------------------------------------
    //! \brief Set the title of the application.
    //! \param p_title The new title of the application.
    // ----------------------------------------------------------------------------
    virtual void setTitle(std::string const& p_title) = 0;

private:

    // ----------------------------------------------------------------------------
    //! \brief Check if the application shall be halted (e.g. by user request or
    //! error during the run() method).
    //! \return true if the application shall be halted, false otherwise.
    // ----------------------------------------------------------------------------
    virtual bool shallBeHalted() const = 0;

    // ----------------------------------------------------------------------------
    //! \brief Initialize the application. Called once at startup. This method
    //! is called before the physics thread is started.
    //! \return true if initialization was successful, false otherwise. In this
    //! last case, you can set the error message using the error() method.
    // ----------------------------------------------------------------------------
    virtual bool setup() = 0;

    // ----------------------------------------------------------------------------
    //! \brief Cleanup the application. Called once at shutdown. This method
    //! is called after the physics thread is stopped.
    //! You can use this method to clean up any resources you allocated in the
    //! onSetup() method.
    // ----------------------------------------------------------------------------
    virtual bool teardown() = 0;

    // ----------------------------------------------------------------------------
    //! \brief Render the application. Called every frame.
    // ----------------------------------------------------------------------------
    virtual void draw() = 0;

    // ----------------------------------------------------------------------------
    //! \brief Update the application logic. Called every target FPS frames.
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
    //! \brief Notify the application of the current FPS.
    //! \param p_fps Current FPS value.
    //! \note This method is called each second.
    // ----------------------------------------------------------------------------
    virtual void onFPSUpdated(size_t const p_fps) = 0;

protected:

    // ----------------------------------------------------------------------------
    //! \brief Handle key input event.
    //! \param p_key Key code.
    //! \param p_scancode Scan code.
    //! \param p_action Action (press, release, repeat).
    //! \param p_mods Modifier keys.
    // ----------------------------------------------------------------------------
    virtual void onKeyInput(int /*p_key*/,
                            int /*p_scancode*/,
                            int /*p_action*/,
                            int /*p_mods*/)
    {
        // Default implementation does nothing
    }

    // ----------------------------------------------------------------------------
    //! \brief Handle mouse button event.
    //! \param p_button Mouse button.
    //! \param p_action Action (press, release).
    //! \param p_mods Modifier keys.
    // ----------------------------------------------------------------------------
    virtual void
    onMouseButton(int /*p_button*/, int /*p_action*/, int /*p_mods*/)
    {
        // Default implementation does nothing
    }

    // ----------------------------------------------------------------------------
    //! \brief Handle cursor position event.
    //! \param p_xpos Cursor x position.
    //! \param p_ypos Cursor y position.
    // ----------------------------------------------------------------------------
    virtual void onCursorPos(double /*p_xpos*/, double /*p_ypos*/)
    {
        // Default implementation does nothing
    }

    // ----------------------------------------------------------------------------
    //! \brief Handle scroll event.
    //! \param p_xoffset Scroll x offset.
    //! \param p_yoffset Scroll y offset.
    // ----------------------------------------------------------------------------
    virtual void onScroll(double /*p_xoffset*/, double /*p_yoffset*/)
    {
        // Default implementation does nothing
    }

    // ----------------------------------------------------------------------------
    //! \brief Handle window resize event.
    //! \param p_width New window width.
    //! \param p_height New window height.
    // ----------------------------------------------------------------------------
    virtual void onWindowResize(int /*p_width*/, int /*p_height*/)
    {
        // Default implementation does nothing
    }

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

protected:

    //! \brief The last error message when run() returned false.
    std::string m_error;

private:

    //! \brief Timing controller for managing rendering FPS and frame timing.
    TimingController m_rendering_timing;
    //! \brief Timing controller for managing physics updates per second.
    TimingController m_physics_timing;
    //! \brief The physics thread.
    //! \note This is used to run the physics simulation in a separate thread.
    std::thread m_physics_thread{};
    //! \brief Whether the physics thread is running.
    //! \note This is used to check if the physics thread is running.
    std::atomic<bool> m_physics_running{ false };
};

} // namespace robotik::viewer::application