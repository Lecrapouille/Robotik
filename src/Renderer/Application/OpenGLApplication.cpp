/**
 * @file OpenGLApplication.cpp
 * @brief OpenGL application base class with ImGui support.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#include "Robotik/Renderer/Application/OpenGLApplication.hpp"
#include "Robotik/Renderer/Application/DearImGuiApplication.hpp"

#include <GL/glew.h>
#include <GLFW/glfw3.h>

namespace robotik::renderer
{

// ----------------------------------------------------------------------------
OpenGLApplication::OpenGLApplication(size_t const p_width,
                                     size_t const p_height,
                                     bool p_enable_imgui)
    : m_width(p_width), m_height(p_height), m_imgui_enabled(p_enable_imgui)
{
}

// ----------------------------------------------------------------------------
OpenGLApplication::~OpenGLApplication()
{
    if (m_window)
    {
        glfwDestroyWindow(m_window);
    }
    glfwTerminate();
}

// ----------------------------------------------------------------------------
GLFWwindow* OpenGLApplication::window() const
{
    return m_window;
}

// ----------------------------------------------------------------------------
void OpenGLApplication::setTitle(std::string const& p_title)
{
    m_title = p_title;
    if (m_window)
    {
        glfwSetWindowTitle(m_window, p_title.c_str());
    }
}

// ----------------------------------------------------------------------------
void OpenGLApplication::halt()
{
    if (m_window)
    {
        glfwSetWindowShouldClose(m_window, GLFW_TRUE);
    }
}

// ----------------------------------------------------------------------------
bool OpenGLApplication::shallBeHalted() const
{
    return m_window ? glfwWindowShouldClose(m_window) : true;
}

// ----------------------------------------------------------------------------
bool OpenGLApplication::setup()
{
    // Initialize GLFW
    if (!initializeGLFW())
        return false;

    // Create window
    if (!createWindow())
        return false;

    // Initialize GLEW
    if (!initializeGlew())
        return false;

    // Setup GLFW callbacks
    setupCallbacks();

    // Initialize OpenGL states
    glEnable(GL_DEPTH_TEST);

    // Initialize ImGui if enabled
    if (m_imgui_enabled)
    {
        m_imgui_app = std::make_unique<DearImGuiApplication>(m_width, m_height);
        if (!m_imgui_app->setup())
        {
            m_error = "Failed to initialize ImGui";
            return false;
        }

        // Set the render callback for the 3D scene
        m_imgui_app->setRenderCallback([this]() { this->onDrawScene(); });

        // Set the ImGui UI callbacks
        m_imgui_app->setMenuBarCallback([this]() { this->onDrawMenuBar(); });
        m_imgui_app->setMainPanelCallback([this]()
                                          { this->onDrawMainPanel(); });
        m_imgui_app->setSidePanelCallback([this]()
                                          { this->onDrawSidePanel(); });
        m_imgui_app->setStatusBarCallback([this]()
                                          { this->onDrawStatusBar(); });
    }

    return onSetup();
}

// ----------------------------------------------------------------------------
bool OpenGLApplication::teardown()
{
    if (m_imgui_app)
    {
        m_imgui_app->teardown();
        m_imgui_app.reset();
    }
    onTeardown();
    return true;
}

// ----------------------------------------------------------------------------
void OpenGLApplication::draw()
{
    // Poll events first
    glfwPollEvents();

    if (m_imgui_enabled && m_imgui_app)
    {
        // Render with ImGui
        m_imgui_app->draw();
    }
    else
    {
        // Direct rendering without ImGui
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        onDrawScene();
    }

    // Swap buffers
    glfwSwapBuffers(m_window);
}

// ----------------------------------------------------------------------------
bool OpenGLApplication::initializeGLFW()
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
bool OpenGLApplication::createWindow()
{
    m_window = glfwCreateWindow(
        int(m_width), int(m_height), m_title.c_str(), nullptr, nullptr);
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
bool OpenGLApplication::initializeGlew()
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
void OpenGLApplication::setupCallbacks()
{
    // Callback resize window
    glfwSetFramebufferSizeCallback(
        m_window,
        [](GLFWwindow* window, int width, int height)
        {
            auto* self = static_cast<OpenGLApplication*>(
                glfwGetWindowUserPointer(window));
            if (self)
            {
                glViewport(0, 0, width, height);
                self->onWindowResize(width, height);
            }
        });

    // Callback keyboard
    glfwSetKeyCallback(
        m_window,
        [](GLFWwindow* window, int key, int scancode, int action, int mods)
        {
            auto* self = static_cast<OpenGLApplication*>(
                glfwGetWindowUserPointer(window));
            if (self)
            {
                self->onKeyInput(key, scancode, action, mods);
            }
        });

    // Callback mouse buttons
    glfwSetMouseButtonCallback(
        m_window,
        [](GLFWwindow* window, int button, int action, int mods)
        {
            auto* self = static_cast<OpenGLApplication*>(
                glfwGetWindowUserPointer(window));
            if (self)
            {
                self->onMouseButton(button, action, mods);
            }
        });

    // Callback mouse position
    glfwSetCursorPosCallback(m_window,
                             [](GLFWwindow* window, double xpos, double ypos)
                             {
                                 auto* self = static_cast<OpenGLApplication*>(
                                     glfwGetWindowUserPointer(window));
                                 if (self)
                                 {
                                     self->onCursorPos(xpos, ypos);
                                 }
                             });

    // Callback scroll wheel
    glfwSetScrollCallback(m_window,
                          [](GLFWwindow* window, double xoffset, double yoffset)
                          {
                              auto* self = static_cast<OpenGLApplication*>(
                                  glfwGetWindowUserPointer(window));
                              if (self)
                              {
                                  self->onScroll(xoffset, yoffset);
                              }
                          });
}

} // namespace robotik::renderer
