/**
 * @file Application.hpp
 * @brief Base application class for ImGui-only applications.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 */

#pragma once

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

#include <string>

namespace robotik::bt {

// ****************************************************************************
//! \brief Base application class for ImGui-only BehaviorTree IDE.
//!
//! This class provides a simplified application framework for applications
//! that only need ImGui rendering without physics simulation.
// ****************************************************************************
class BTApplication
{
public:

    // ------------------------------------------------------------------------
    //! \brief Constructor.
    //! \param p_width Initial window width.
    //! \param p_height Initial window height.
    // ------------------------------------------------------------------------
    BTApplication(size_t const p_width, size_t const p_height);

    // ------------------------------------------------------------------------
    //! \brief Destructor.
    // ------------------------------------------------------------------------
    virtual ~BTApplication();

    // ------------------------------------------------------------------------
    //! \brief Run the application main loop.
    //! \return true if the application ran successfully, false on error.
    // ------------------------------------------------------------------------
    bool run();

    // ------------------------------------------------------------------------
    //! \brief Get the last error message.
    //! \return The error message or empty string if no error.
    // ------------------------------------------------------------------------
    std::string const& error() const
    {
        return m_error;
    }

    // ------------------------------------------------------------------------
    //! \brief Set the window title.
    //! \param p_title The new window title.
    // ------------------------------------------------------------------------
    void setTitle(std::string const& p_title);

    // ------------------------------------------------------------------------
    //! \brief Request the application to stop.
    // ------------------------------------------------------------------------
    void halt()
    {
        m_running = false;
    }

protected:

    // ------------------------------------------------------------------------
    //! \brief Called once at startup to initialize the application.
    //! \return true if initialization was successful.
    // ------------------------------------------------------------------------
    virtual bool onSetup() = 0;

    // ------------------------------------------------------------------------
    //! \brief Called once at shutdown to cleanup resources.
    // ------------------------------------------------------------------------
    virtual void onTeardown() = 0;

    // ------------------------------------------------------------------------
    //! \brief Called every frame to update application state.
    //! \param p_dt Delta time since last frame.
    // ------------------------------------------------------------------------
    virtual void onUpdate(float p_dt) = 0;

    // ------------------------------------------------------------------------
    //! \brief Called to draw the menu bar.
    // ------------------------------------------------------------------------
    virtual void onDrawMenuBar() {}

    // ------------------------------------------------------------------------
    //! \brief Called to draw the main panel.
    // ------------------------------------------------------------------------
    virtual void onDrawMainPanel() {}

    // ------------------------------------------------------------------------
    //! \brief Called to draw the side panel.
    // ------------------------------------------------------------------------
    virtual void onDrawSidePanel() {}

    // ------------------------------------------------------------------------
    //! \brief Called to draw the status bar.
    // ------------------------------------------------------------------------
    virtual void onDrawStatusBar() {}

private:

    // ------------------------------------------------------------------------
    //! \brief Initialize GLFW window and OpenGL context.
    //! \return true if initialization was successful.
    // ------------------------------------------------------------------------
    bool initWindow();

    // ------------------------------------------------------------------------
    //! \brief Initialize ImGui context.
    //! \return true if initialization was successful.
    // ------------------------------------------------------------------------
    bool initImGui();

    // ------------------------------------------------------------------------
    //! \brief Cleanup ImGui resources.
    // ------------------------------------------------------------------------
    void shutdownImGui();

    // ------------------------------------------------------------------------
    //! \brief Cleanup GLFW resources.
    // ------------------------------------------------------------------------
    void shutdownWindow();

    // ------------------------------------------------------------------------
    //! \brief Process a single frame.
    // ------------------------------------------------------------------------
    void processFrame();

    // ------------------------------------------------------------------------
    //! \brief Setup the dockspace for ImGui.
    // ------------------------------------------------------------------------
    void setupDockspace();

protected:

    //! \brief Error message
    std::string m_error;

private:

    //! \brief GLFW window handle
    GLFWwindow* m_window = nullptr;
    //! \brief Window width
    size_t m_width;
    //! \brief Window height
    size_t m_height;
    //! \brief Last frame time
    double m_last_time = 0.0;
    //! \brief Running state
    bool m_running = true;
};

} // namespace robotik::bt
