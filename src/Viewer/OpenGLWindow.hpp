/**
 * @file OpenGLWindow.hpp
 * @brief OpenGL window base class for the 3D viewer.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#pragma once

#include <functional>
#include <string>

class GLFWwindow;

namespace robotik::viewer
{

// Forward declaration
struct WindowConfig
{
    size_t window_width = 800;
    size_t window_height = 600;
    std::string window_title = "Robot Viewer";
};

// ****************************************************************************
//! \brief Simple OpenGL viewer for robot visualization.
//!
//! This viewer provides basic 3D visualization capabilities:
//! - Camera positioning (top, side, perspective views)
//! - Basic geometry rendering (box, cylinder, sphere)
//! - Grid floor
//! - Robot arm visualization
//! - No lighting (flat shading)
// ****************************************************************************
class OpenGLWindow
{
public:

    // Callback types
    using KeyCallback = std::function<void(int, int, int, int)>;
    using MouseButtonCallback = std::function<void(int, int, int)>;
    using CursorPosCallback = std::function<void(double, double)>;
    using ScrollCallback = std::function<void(double, double)>;
    using WindowResizeCallback = std::function<void(int, int)>;

    // ------------------------------------------------------------------------
    //! \brief Constructor.
    //! \param p_config Configuration for the window.
    // ------------------------------------------------------------------------
    explicit OpenGLWindow(const WindowConfig& p_config);

    // ------------------------------------------------------------------------
    //! \brief Destructor.
    // ------------------------------------------------------------------------
    ~OpenGLWindow();

    // ------------------------------------------------------------------------
    //! \brief Initialize the viewer.
    //! \return true if initialization successful.
    // ------------------------------------------------------------------------
    bool initialize();

    // ------------------------------------------------------------------------
    //! \brief Check if window should close.
    //! \return true if window should close.
    // ------------------------------------------------------------------------
    bool isHalting() const;

    // ------------------------------------------------------------------------
    //! \brief Set the should close flag.
    //! \param p_should_close Should close flag.
    // ------------------------------------------------------------------------
    void halt();

    // ------------------------------------------------------------------------
    //! \brief Set the title of the window.
    //! \param p_title Title of the window.
    // ------------------------------------------------------------------------
    void setTitle(std::string const& p_title);

    // ------------------------------------------------------------------------
    //! \brief Swap the buffers.
    // ------------------------------------------------------------------------
    void swapBuffers();

    // ------------------------------------------------------------------------
    //! \brief Get the GLFW window.
    //! \return Pointer to GLFW window.
    // ------------------------------------------------------------------------
    GLFWwindow* window() const;

    // ------------------------------------------------------------------------
    //! \brief Get the error message.
    //! \return Error message.
    // ------------------------------------------------------------------------
    std::string const& error() const;

    // ------------------------------------------------------------------------
    //! \brief Set the callbacks for window events.
    //! \param p_key_callback Key callback.
    //! \param p_mouse_button_callback Mouse button callback.
    //! \param p_cursor_pos_callback Cursor position callback.
    //! \param p_scroll_callback Scroll callback.
    //! \param p_window_resize_callback Window resize callback.
    // ------------------------------------------------------------------------
    void setCallbacks(KeyCallback const& p_key_callback,
                      MouseButtonCallback const& p_mouse_button_callback,
                      CursorPosCallback const& p_cursor_pos_callback,
                      ScrollCallback const& p_scroll_callback,
                      WindowResizeCallback const& p_window_resize_callback);

private:

    bool initializeGLFW();
    bool createWindow();
    bool initializeGlew();
    void setupCallbacks();

private:

    // Window properties
    size_t m_width;
    size_t m_height;
    GLFWwindow* m_window = nullptr;
    std::string m_error;

    // Callbacks
    KeyCallback m_key_callback;
    MouseButtonCallback m_mouse_button_callback;
    CursorPosCallback m_cursor_pos_callback;
    ScrollCallback m_scroll_callback;
    WindowResizeCallback m_window_resize_callback;
};

} // namespace robotik::viewer