/**
 * @file OpenGLWindow.cpp
 * @brief OpenGL window class for the 3D viewer.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#include "Viewer/OpenGLWindow.hpp"
#include "Viewer/RobotViewerApplication.hpp"

#include <GL/glew.h>
#include <GLFW/glfw3.h>

namespace robotik::viewer
{
// ----------------------------------------------------------------------------
OpenGLWindow::OpenGLWindow(Configuration const& p_config)
    : m_width(p_config.window_width), m_height(p_config.window_height)
{
}

// ----------------------------------------------------------------------------
OpenGLWindow::~OpenGLWindow()
{
    if (m_window)
    {
        glfwDestroyWindow(m_window);
    }
    glfwTerminate();
}

// ----------------------------------------------------------------------------
GLFWwindow* OpenGLWindow::window() const
{
    return m_window;
}

// ----------------------------------------------------------------------------
void OpenGLWindow::swapBuffers()
{
    glfwSwapBuffers(m_window);
}

// ----------------------------------------------------------------------------
bool OpenGLWindow::initialize(
    KeyCallback const& p_key_callback,
    MouseButtonCallback const& p_mouse_button_callback,
    CursorPosCallback const& p_cursor_pos_callback,
    ScrollCallback const& p_scroll_callback,
    WindowResizeCallback const& p_window_resize_callback)
{
    if (!initializeGLFW())
        return false;

    if (!createWindow())
        return false;

    if (!initializeGlew())
        return false;

    setupCallbacks(p_key_callback,
                   p_mouse_button_callback,
                   p_cursor_pos_callback,
                   p_scroll_callback,
                   p_window_resize_callback);

    return true;
}

// ----------------------------------------------------------------------------
bool OpenGLWindow::initializeGLFW()
{
    if (!glfwInit())
    {
        m_error = "Failed to initialize GLFW";
        return false;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // Mac OS X
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_SAMPLES, 4); // 4x antialiasing

    return true;
}

// ----------------------------------------------------------------------------
bool OpenGLWindow::createWindow()
{
    m_window =
        glfwCreateWindow(int(m_width), int(m_height), "", nullptr, nullptr);
    if (!m_window)
    {
        m_error = "Failed to create GLFW window";
        glfwTerminate();
        return false;
    }

    glfwMakeContextCurrent(m_window);
    glfwSetWindowUserPointer(m_window, this);

    glfwSwapInterval(1);

    return true;
}

// ----------------------------------------------------------------------------
bool OpenGLWindow::initializeGlew()
{
    // Initialize GLEW
    glewExperimental = GL_TRUE;
    if (glewInit() != GLEW_OK)
    {
        m_error = "Failed to initialize GLEW";
        glfwTerminate();
        return false;
    }

    // Make sure OpenGL version 3.3 API is available
    if (!GLEW_VERSION_3_3)
    {
        m_error = "OpenGL 3.3 not supported";
        glfwTerminate();
        return false;
    }

    return true;
}

// ----------------------------------------------------------------------------
void OpenGLWindow::setupCallbacks(
    KeyCallback const& p_key_callback,
    MouseButtonCallback const& p_mouse_button_callback,
    CursorPosCallback const& p_cursor_pos_callback,
    ScrollCallback const& p_scroll_callback,
    WindowResizeCallback const& p_window_resize_callback)
{
    // Callback resize window
    m_window_resize_callback = p_window_resize_callback;
    glfwSetFramebufferSizeCallback(
        m_window,
        [](GLFWwindow* window, int width, int height)
        {
            auto* self =
                static_cast<OpenGLWindow*>(glfwGetWindowUserPointer(window));
            if (self->m_window_resize_callback)
            {
                self->m_window_resize_callback(width, height);
            }
        });

    // Callback keyboard
    m_key_callback = p_key_callback;
    glfwSetKeyCallback(
        m_window,
        [](GLFWwindow* window, int key, int scancode, int action, int mods)
        {
            auto* self =
                static_cast<OpenGLWindow*>(glfwGetWindowUserPointer(window));
            if (self->m_key_callback)
            {
                self->m_key_callback(key, scancode, action, mods);
            }
        });

    // Callback mouse buttons
    m_mouse_button_callback = p_mouse_button_callback;
    m_key_callback = p_key_callback;
    glfwSetMouseButtonCallback(
        m_window,
        [](GLFWwindow* window, int button, int action, int mods)
        {
            auto* self =
                static_cast<OpenGLWindow*>(glfwGetWindowUserPointer(window));
            if (self->m_mouse_button_callback)
            {
                self->m_mouse_button_callback(button, action, mods);
            }
        });

    // Callback mouse position
    m_cursor_pos_callback = p_cursor_pos_callback;
    glfwSetCursorPosCallback(m_window,
                             [](GLFWwindow* window, double xpos, double ypos)
                             {
                                 auto* self = static_cast<OpenGLWindow*>(
                                     glfwGetWindowUserPointer(window));
                                 if (self->m_cursor_pos_callback)
                                 {
                                     self->m_cursor_pos_callback(xpos, ypos);
                                 }
                             });

    // Callback scroll wheel
    m_scroll_callback = p_scroll_callback;
    glfwSetScrollCallback(m_window,
                          [](GLFWwindow* window, double xoffset, double yoffset)
                          {
                              auto* self = static_cast<OpenGLWindow*>(
                                  glfwGetWindowUserPointer(window));
                              if (self->m_scroll_callback)
                              {
                                  self->m_scroll_callback(xoffset, yoffset);
                              }
                          });
}

// ----------------------------------------------------------------------------
bool OpenGLWindow::isHalting() const
{
    return glfwWindowShouldClose(m_window);
}

// ----------------------------------------------------------------------------
void OpenGLWindow::halt()
{
    glfwSetWindowShouldClose(m_window, GLFW_TRUE);
}

// ----------------------------------------------------------------------------
void OpenGLWindow::setTitle(std::string const& p_title)
{
    if (!m_window)
        return;

    glfwSetWindowTitle(m_window, p_title.c_str());
}

// ----------------------------------------------------------------------------
std::string const& OpenGLWindow::error() const
{
    return m_error;
}

} // namespace robotik::viewer