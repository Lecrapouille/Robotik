/**
 * @file Application.hpp
 * @brief Application base class for the viewer.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#pragma once

#include "Viewer/OpenGLWindow.hpp"

#include "Robotik/private/Path.hpp"

#include <atomic>
#include <thread>

namespace robotik
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
    //! \brief Constructor. No actions are performed here.
    //! \param p_path Path searcher.
    //! \param p_window_width Window width in pixels.
    //! \param p_window_height Window height in pixels.
    //! \param p_window_title Window title.
    // ----------------------------------------------------------------------------
    Application(Path& p_path,
                size_t p_window_width,
                size_t p_window_height,
                const std::string& p_window_title);

    // ----------------------------------------------------------------------------
    //! \brief Destructor.
    // ----------------------------------------------------------------------------
    virtual ~Application();

    // ----------------------------------------------------------------------------
    //! \brief Start the application, start the physics thread and run the main
    //! loop.
    //! \param p_target_fps Target rendering frame rate (default: 60 FPS).
    //! \param p_target_physics_hz Target physics update rate (default: 1000
    //! Hz).
    //! \return true if the application should close, false otherwise.
    // ----------------------------------------------------------------------------
    bool run(size_t const p_target_fps = 60u,
             size_t const p_target_physics_hz = 1000u);

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
    //! \brief Handle input events. Called for each input event.
    //! \param p_key The key that was pressed.
    //! \param p_action The action (press/release).
    // ----------------------------------------------------------------------------
    virtual void handleInput(int p_key, int p_action) = 0;

private:

    // ----------------------------------------------------------------------------
    //! \brief Start the physics thread.
    // ----------------------------------------------------------------------------
    void startPhysicsThread(size_t const p_target_physics_hz);

    // ----------------------------------------------------------------------------
    //! \brief Stop the physics thread.
    // ----------------------------------------------------------------------------
    void stopPhysicsThread();

    // ----------------------------------------------------------------------------
    //! \brief Check if a key is pressed.
    //! \param p_key The key to check.
    //! \return true if the key is pressed, false otherwise.
    // ----------------------------------------------------------------------------
    bool isKeyPressed(int p_key) const;

protected:

    OpenGLWindow m_viewer;

private:

    std::thread m_physics_thread{};
    std::atomic<bool> m_physics_running{ false };
    std::string m_error;
};

} // namespace robotik