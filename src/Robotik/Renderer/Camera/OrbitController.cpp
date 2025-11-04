/**
 * @file OrbitController.cpp
 * @brief Orbit camera controller implementation.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#include "Robotik/Renderer/Camera/OrbitController.hpp"

#include <GLFW/glfw3.h>
#include <cmath>

namespace robotik::renderer
{

// ----------------------------------------------------------------------------
OrbitController::OrbitController(Camera& p_camera,
                                 const Eigen::Vector3f& p_target,
                                 float p_distance)
    : CameraController(p_camera),
      m_target(p_target),
      m_distance(p_distance),
      m_azimuth(45.0f * M_PIf / 180.0f),
      m_elevation(30.0f * M_PIf / 180.0f),
      m_sensitivity(0.3f),
      m_zoom_sensitivity(0.1f),
      m_pan_sensitivity(0.01f),
      m_min_distance(0.5f),
      m_max_distance(100.0f),
      m_min_elevation(-89.0f * M_PIf / 180.0f),
      m_max_elevation(89.0f * M_PIf / 180.0f),
      m_is_orbiting(false),
      m_is_panning(false),
      m_last_mouse_x(0.0),
      m_last_mouse_y(0.0)
{
    updateCameraPosition();
}

// ----------------------------------------------------------------------------
void OrbitController::setTarget(const Eigen::Vector3f& p_target)
{
    m_target = p_target;
    updateCameraPosition();
}

// ----------------------------------------------------------------------------
void OrbitController::setDistance(float p_distance)
{
    m_distance = std::clamp(p_distance, m_min_distance, m_max_distance);
    updateCameraPosition();
}

// ----------------------------------------------------------------------------
void OrbitController::setSensitivity(float p_sensitivity)
{
    m_sensitivity = p_sensitivity;
}

// ----------------------------------------------------------------------------
void OrbitController::setZoomSensitivity(float p_sensitivity)
{
    m_zoom_sensitivity = p_sensitivity;
}

// ----------------------------------------------------------------------------
void OrbitController::setPanSensitivity(float p_sensitivity)
{
    m_pan_sensitivity = p_sensitivity;
}

// ----------------------------------------------------------------------------
void OrbitController::update(float /*p_dt*/)
{
    // Update camera if needed
    m_camera.update(0.0f);
}

// ----------------------------------------------------------------------------
void OrbitController::handleMouseMove(double p_xpos, double p_ypos)
{
    double dx = p_xpos - m_last_mouse_x;
    double dy = p_ypos - m_last_mouse_y;
    m_last_mouse_x = p_xpos;
    m_last_mouse_y = p_ypos;

    if (m_is_orbiting)
    {
        // Orbit around target
        m_azimuth -= static_cast<float>(dx) * m_sensitivity * 0.01f;
        m_elevation += static_cast<float>(dy) * m_sensitivity * 0.01f;

        // Clamp elevation to avoid gimbal lock
        m_elevation = std::clamp(m_elevation, m_min_elevation, m_max_elevation);

        updateCameraPosition();
    }
    else if (m_is_panning)
    {
        // Pan target point
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
void OrbitController::handleMouseButton(int p_button,
                                        int p_action,
                                        int /*p_mods*/)
{
    if (p_button == GLFW_MOUSE_BUTTON_LEFT)
    {
        if (p_action == GLFW_PRESS)
        {
            m_is_orbiting = true;
        }
        else if (p_action == GLFW_RELEASE)
        {
            m_is_orbiting = false;
        }
    }
    else if (p_button == GLFW_MOUSE_BUTTON_RIGHT ||
             p_button == GLFW_MOUSE_BUTTON_MIDDLE)
    {
        if (p_action == GLFW_PRESS)
        {
            m_is_panning = true;
        }
        else if (p_action == GLFW_RELEASE)
        {
            m_is_panning = false;
        }
    }
}

// ----------------------------------------------------------------------------
void OrbitController::handleScroll(double /*p_xoffset*/, double p_yoffset)
{
    // Zoom in/out
    m_distance -=
        static_cast<float>(p_yoffset) * m_zoom_sensitivity * m_distance;
    m_distance = std::clamp(m_distance, m_min_distance, m_max_distance);

    updateCameraPosition();
}

// ----------------------------------------------------------------------------
void OrbitController::handleKeyboard(int /*p_key*/,
                                     int /*p_action*/,
                                     int /*p_mods*/)
{
    // No keyboard controls for orbit controller
}

// ----------------------------------------------------------------------------
void OrbitController::reset()
{
    m_target = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
    m_distance = 5.0f;
    m_azimuth = 45.0f * M_PIf / 180.0f;
    m_elevation = 30.0f * M_PIf / 180.0f;
    m_is_orbiting = false;
    m_is_panning = false;

    updateCameraPosition();
}

// ----------------------------------------------------------------------------
bool OrbitController::isUserInteracting() const
{
    return m_is_orbiting || m_is_panning;
}

// ----------------------------------------------------------------------------
void OrbitController::updateCameraPosition()
{
    // Convert spherical coordinates to Cartesian
    float x = m_distance * std::cos(m_elevation) * std::cos(m_azimuth);
    float y = m_distance * std::cos(m_elevation) * std::sin(m_azimuth);
    float z = m_distance * std::sin(m_elevation);

    Eigen::Vector3f position = m_target + Eigen::Vector3f(x, y, z);

    // Update camera
    m_camera.lookAt(position, m_target, Eigen::Vector3f(0.0f, 0.0f, 1.0f));
}

} // namespace robotik::renderer
