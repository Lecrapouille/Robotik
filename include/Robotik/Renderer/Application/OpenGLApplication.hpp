/**
 * @file OpenGLApplication.hpp
 * @brief OpenGL application base class with ImGui support.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#pragma once

#include "Robotik/Renderer/Application/Application.hpp"

#include <memory>
#include <string>

struct GLFWwindow;

namespace robotik::renderer::application
{

// Forward declaration
class DearImGuiApplication;

// ****************************************************************************
//! \brief OpenGL application with optional ImGui integration.
//!
//! This class provides:
//! - OpenGL window management via GLFW.
//! - Optional ImGui integration for UI.
//! - Event handling (keyboard, mouse, scroll, resize).
//! - Automatic setup and teardown of OpenGL and ImGui.
//!
//! Users should inherit from this class and implement:
//! - onDrawScene() for 3D rendering
//! - Optional ImGui methods: onDrawMenuBar(), onDrawMainPanel(), etc.
//! - Optional event handlers: onKeyInput(), onMouseButton(), etc.
// ****************************************************************************
class OpenGLApplication: public Application
{
public:

    // ------------------------------------------------------------------------
    //! \brief Constructor.
    //! \param p_width Initial window width.
    //! \param p_height Initial window height.
    //! \param p_enable_imgui Enable ImGui UI (default: true).
    // ------------------------------------------------------------------------
    explicit OpenGLApplication(size_t const p_width,
                               size_t const p_height,
                               bool p_enable_imgui = true);

    // ------------------------------------------------------------------------
    //! \brief Destructor.
    // ------------------------------------------------------------------------
    ~OpenGLApplication() override;

    // ------------------------------------------------------------------------
    //! \brief Set the title of the application window.
    //! \param p_title The title of the application.
    // ------------------------------------------------------------------------
    void setTitle(std::string const& p_title) override;

    // ------------------------------------------------------------------------
    //! \brief Get the GLFW window.
    //! \return Pointer to GLFW window.
    // ------------------------------------------------------------------------
    GLFWwindow* window() const;

    // ------------------------------------------------------------------------
    //! \brief Request the application to close.
    // ------------------------------------------------------------------------
    void halt();

protected:

    // ------------------------------------------------------------------------
    //! \brief Initialize the application. Called once at startup. This method
    //! is called before the physics thread is started.
    //! \return true if initialization was successful, false otherwise. In this
    //! last case, you can set the error message using the error() method.
    // ------------------------------------------------------------------------
    virtual bool onSetup() = 0;

    // ------------------------------------------------------------------------
    //! \brief Cleanup the application. Called once at shutdown. This method
    //! is called after the physics thread is stopped.
    //! You can use this method to clean up any resources you allocated in the
    //! onSetup() method.
    // ------------------------------------------------------------------------
    virtual void onTeardown() = 0;

    // ------------------------------------------------------------------------
    //! \brief Render the 3D scene. Override this to draw your 3D content.
    //! This is called automatically within the ImGui viewport or directly.
    // ------------------------------------------------------------------------
    virtual void onDrawScene()
    {
        // Empty implementation
    }

    // ------------------------------------------------------------------------
    //! \brief Draw ImGui menu bar. Override to add custom menu items.
    // ------------------------------------------------------------------------
    virtual void onDrawMenuBar()
    {
        // Empty implementation
    }

    // ------------------------------------------------------------------------
    //! \brief Draw ImGui main control panel. Override to add custom controls.
    // ------------------------------------------------------------------------
    virtual void onDrawMainPanel()
    {
        // Empty implementation
    }

    // ------------------------------------------------------------------------
    //! \brief Draw ImGui side panel. Override to add side panel content.
    // ------------------------------------------------------------------------
    virtual void onDrawSidePanel()
    {
        // Empty implementation
    }

    // ------------------------------------------------------------------------
    //! \brief Draw ImGui status bar. Override to add status bar content.
    // ------------------------------------------------------------------------
    virtual void onDrawStatusBar()
    {
        // Empty implementation
    }

private:

    // Override Application methods
    bool shallBeHalted() const final;
    bool setup() final;
    bool teardown() final;
    void draw() final;

    // Internal initialization methods
    bool initializeGLFW();
    bool createWindow();
    bool initializeGlew();
    void setupCallbacks();

private:

    // Window properties
    size_t m_width;
    size_t m_height;
    GLFWwindow* m_window = nullptr;
    std::string m_title;

    // ImGui integration
    bool m_imgui_enabled;
    std::unique_ptr<DearImGuiApplication> m_imgui_app;
};

} // namespace robotik::renderer::application
