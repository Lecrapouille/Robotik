/**
 * @file RobotViewerApplication.cpp
 * @brief Robot viewer application class for the 3D viewer.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#include "Viewer/RobotViewerApplication.hpp"

#include <GLFW/glfw3.h>

namespace robotik::viewer
{

// ----------------------------------------------------------------------------
RobotViewerApplication::RobotViewerApplication(Configuration& p_config)
    : m_config(p_config),
      m_window(WindowConfig{ p_config.window_width,
                             p_config.window_height,
                             p_config.window_title }),
      m_robot_viewer(p_config.window_width, p_config.window_height),
      m_title(p_config.window_title)
{
    m_window.setTitle(m_title + " - FPS: " + std::to_string(m_fps));
}

// ----------------------------------------------------------------------------
RobotViewerApplication::~RobotViewerApplication() = default;

// ----------------------------------------------------------------------------
bool RobotViewerApplication::run()
{
    return Application::run(m_config.target_fps, m_config.target_physics_hz);
}

// ----------------------------------------------------------------------------
void RobotViewerApplication::setTitle(std::string const& p_title)
{
    m_title = p_title;
    m_window.setTitle(m_title + " - FPS: " + std::to_string(m_fps));
}

// ----------------------------------------------------------------------------
bool RobotViewerApplication::isHalting() const
{
    return m_window.isHalting();
}

// ----------------------------------------------------------------------------
bool RobotViewerApplication::onSetup()
{
    if (!m_window.initialize())
    {
        m_error = m_window.error();
        return false;
    }

    m_window.setCallbacks(std::bind(&RobotViewerApplication::onKeyInput,
                                    this,
                                    std::placeholders::_1,
                                    std::placeholders::_2,
                                    std::placeholders::_3,
                                    std::placeholders::_4),
                          std::bind(&RobotViewerApplication::onMouseButton,
                                    this,
                                    std::placeholders::_1,
                                    std::placeholders::_2,
                                    std::placeholders::_3),
                          std::bind(&RobotViewerApplication::onCursorPos,
                                    this,
                                    std::placeholders::_1,
                                    std::placeholders::_2),
                          std::bind(&RobotViewerApplication::onScroll,
                                    this,
                                    std::placeholders::_1,
                                    std::placeholders::_2),
                          std::bind(&RobotViewerApplication::onWindowResize,
                                    this,
                                    std::placeholders::_1,
                                    std::placeholders::_2));

    // Enable depth testing
    glEnable(GL_DEPTH_TEST);

    return true;
}

// ----------------------------------------------------------------------------
void RobotViewerApplication::onCleanup() {}

// ----------------------------------------------------------------------------
void RobotViewerApplication::onDraw()
{
    m_window.swapBuffers();
}

// ----------------------------------------------------------------------------
void RobotViewerApplication::onUpdate(float const /* dt */) {}

// ----------------------------------------------------------------------------
void RobotViewerApplication::onHandleEvents()
{
    // Poll GLFW events to process callbacks
    glfwPollEvents();
}

// ----------------------------------------------------------------------------
void RobotViewerApplication::onPhysicUpdate(float const /* dt */) {}

// ----------------------------------------------------------------------------
void RobotViewerApplication::onFPSUpdated(size_t const p_fps)
{
    m_fps = p_fps;
    m_window.setTitle(m_title + " - FPS: " + std::to_string(p_fps));
}

// ----------------------------------------------------------------------------
void RobotViewerApplication::onWindowResize(int width, int height)
{
    glViewport(0, 0, width, height);
}

// ----------------------------------------------------------------------------
void RobotViewerApplication::onKeyInput(int key,
                                        int /* scancode */,
                                        int action,
                                        int /* mods */)
{
    // Handle key event
    if (action == GLFW_PRESS)
    {
        switch (key)
        {
            case GLFW_KEY_ESCAPE:
                m_window.halt();
                break;
            default:
                break;
        }
    }
}

// ----------------------------------------------------------------------------
void RobotViewerApplication::onMouseButton(int /* button */,
                                           int /* action */,
                                           int /* mods */)
{
    // Handle mouse button event
    // This could be used for camera controls, object selection, etc.
}

// ----------------------------------------------------------------------------
void RobotViewerApplication::onCursorPos(double /* xpos */, double /* ypos */)
{
    // Handle cursor position event
    // This could be used for camera controls, mouse tracking, etc.
}

// ----------------------------------------------------------------------------
void RobotViewerApplication::onScroll(double /* xoffset */,
                                      double /* yoffset */)
{
    // Handle scroll event
    // This could be used for zoom controls, etc.
}

} // namespace robotik::viewer