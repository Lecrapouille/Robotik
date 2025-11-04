/**
 * @file DragController.cpp
 * @brief Drag camera controller implementation.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#include "Robotik/Renderer/Camera/DragController.hpp"

#include <GLFW/glfw3.h>
#include <algorithm>

namespace robotik::renderer
{

// ----------------------------------------------------------------------------
DragController::DragController(Camera& p_camera,
                               const Eigen::Vector3f& p_target,
                               float p_distance)
    : CameraController(p_camera),
      m_target(p_target),
      m_distance(p_distance),
      m_direction(0.0f, 0.0f, 1.0f), // Default: looking down from above
      m_pan_sensitivity(0.01f),
      m_zoom_sensitivity(0.1f),
      m_min_distance(0.5f),
      m_max_distance(100.0f),
      m_is_dragging(false),
      m_last_mouse_x(0.0),
      m_last_mouse_y(0.0)
{
    // Calculate direction from current camera position if available
    Eigen::Vector3f current_pos = p_camera.position();
    Eigen::Vector3f current_target = p_camera.target();
    Eigen::Vector3f dir = current_pos - current_target;
    float current_distance = dir.norm();

    if (current_distance > 0.1f)
    {
        // Use current camera direction
        m_direction = dir.normalized();
        m_distance = current_distance;
    }

    updateCameraPosition();
}

// ----------------------------------------------------------------------------
void DragController::setPanSensitivity(float p_sensitivity)
{
    m_pan_sensitivity = p_sensitivity;
}

// ----------------------------------------------------------------------------
void DragController::setZoomSensitivity(float p_sensitivity)
{
    m_zoom_sensitivity = p_sensitivity;
}

// ----------------------------------------------------------------------------
void DragController::setTarget(const Eigen::Vector3f& p_target)
{
    m_target = p_target;
    updateCameraPosition();
}

// ----------------------------------------------------------------------------
void DragController::update(float /*p_dt*/)
{
    m_camera.update(0.0f);
}

// ----------------------------------------------------------------------------
void DragController::handleMouseMove(double p_xpos, double p_ypos)
{
    double dx = p_xpos - m_last_mouse_x;
    double dy = p_ypos - m_last_mouse_y;
    m_last_mouse_x = p_xpos;
    m_last_mouse_y = p_ypos;

    if (m_is_dragging)
    {
        // Pan the target point
        Eigen::Vector3f right = m_camera.right();
        Eigen::Vector3f up = m_camera.up();

        m_target -=
            right * static_cast<float>(dx) * m_pan_sensitivity * m_distance;
        m_target +=
            up * static_cast<float>(dy) * m_pan_sensitivity * m_distance;

        updateCameraPosition();
    }
}

// ----------------------------------------------------------------------------
void DragController::handleMouseButton(int p_button,
                                       int p_action,
                                       int /*p_mods*/)
{
    if (p_button == GLFW_MOUSE_BUTTON_LEFT ||
        p_button == GLFW_MOUSE_BUTTON_MIDDLE)
    {
        if (p_action == GLFW_PRESS)
        {
            m_is_dragging = true;
        }
        else if (p_action == GLFW_RELEASE)
        {
            m_is_dragging = false;
        }
    }
}

// ----------------------------------------------------------------------------
void DragController::handleScroll(double /*p_xoffset*/, double p_yoffset)
{
    // Zoom in/out
    m_distance -=
        static_cast<float>(p_yoffset) * m_zoom_sensitivity * m_distance;
    m_distance = std::clamp(m_distance, m_min_distance, m_max_distance);

    updateCameraPosition();
}

// ----------------------------------------------------------------------------
void DragController::handleKeyboard(int /*p_key*/,
                                    int /*p_action*/,
                                    int /*p_mods*/)
{
    // No keyboard controls for drag controller
}

// ----------------------------------------------------------------------------
void DragController::reset()
{
    m_target = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
    m_distance = 10.0f;
    m_is_dragging = false;

    updateCameraPosition();
}

// ----------------------------------------------------------------------------
void DragController::updateCameraPosition()
{
    // Calculate camera position using stored direction and distance
    // This preserves the viewing angle (top, front, side, etc.)
    Eigen::Vector3f position = m_target + m_direction * m_distance;

    // Determine up vector based on direction
    // For most views, use (0, 0, 1) as up, but for top/bottom views use (0, 1,
    // 0)
    Eigen::Vector3f up;
    if (std::abs(m_direction.z()) > 0.9f)
    {
        // Looking mostly up or down (top/bottom view)
        up = Eigen::Vector3f(0.0f, 1.0f, 0.0f);
    }
    else
    {
        // Side views (front, back, left, right)
        up = Eigen::Vector3f(0.0f, 0.0f, 1.0f);
    }

    m_camera.lookAt(position, m_target, up);
}

} // namespace robotik::renderer
