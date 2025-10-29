/**
 * @file FlyController.cpp
 * @brief Fly camera controller implementation.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#include "Robotik/Renderer/Camera/FlyController.hpp"

#include <GLFW/glfw3.h>
#include <cmath>

namespace robotik::renderer
{

// ----------------------------------------------------------------------------
FlyController::FlyController(Camera& p_camera, float p_move_speed)
    : CameraController(p_camera),
      m_move_speed(p_move_speed),
      m_look_sensitivity(0.1f),
      m_speed_multiplier(2.0f),
      m_yaw(0.0f),
      m_pitch(0.0f),
      m_move_forward(false),
      m_move_backward(false),
      m_move_left(false),
      m_move_right(false),
      m_move_up(false),
      m_move_down(false),
      m_fast_mode(false),
      m_is_looking(false),
      m_last_mouse_x(0.0),
      m_last_mouse_y(0.0),
      m_first_mouse(true)
{
    // Initialize yaw and pitch from camera's current direction
    Eigen::Vector3f forward = m_camera.forward();
    m_yaw = std::atan2(forward.y(), forward.x()) * 180.0f / M_PI;
    m_pitch = std::asin(forward.z()) * 180.0f / M_PI;
}

// ----------------------------------------------------------------------------
void FlyController::setMoveSpeed(float p_speed)
{
    m_move_speed = p_speed;
}

// ----------------------------------------------------------------------------
void FlyController::setLookSensitivity(float p_sensitivity)
{
    m_look_sensitivity = p_sensitivity;
}

// ----------------------------------------------------------------------------
void FlyController::update(float p_dt)
{
    // Calculate movement
    Eigen::Vector3f movement(0.0f, 0.0f, 0.0f);
    float speed = m_move_speed * (m_fast_mode ? m_speed_multiplier : 1.0f);

    if (m_move_forward)
        movement += m_camera.forward() * speed * p_dt;
    if (m_move_backward)
        movement -= m_camera.forward() * speed * p_dt;
    if (m_move_right)
        movement += m_camera.right() * speed * p_dt;
    if (m_move_left)
        movement -= m_camera.right() * speed * p_dt;
    if (m_move_up)
        movement += m_camera.up() * speed * p_dt;
    if (m_move_down)
        movement -= m_camera.up() * speed * p_dt;

    // Update camera position
    if (movement.norm() > 0.0f)
    {
        Eigen::Vector3f new_pos = m_camera.position() + movement;
        m_camera.setPosition(new_pos);
    }

    m_camera.update(p_dt);
}

// ----------------------------------------------------------------------------
void FlyController::handleMouseMove(double p_xpos, double p_ypos)
{
    if (m_first_mouse)
    {
        m_last_mouse_x = p_xpos;
        m_last_mouse_y = p_ypos;
        m_first_mouse = false;
        return;
    }

    if (m_is_looking)
    {
        double dx = p_xpos - m_last_mouse_x;
        double dy = p_ypos - m_last_mouse_y;

        m_yaw += static_cast<float>(dx) * m_look_sensitivity;
        m_pitch -= static_cast<float>(dy) * m_look_sensitivity;

        // Constrain pitch
        if (m_pitch > 89.0f)
            m_pitch = 89.0f;
        if (m_pitch < -89.0f)
            m_pitch = -89.0f;

        updateCameraOrientation();
    }

    m_last_mouse_x = p_xpos;
    m_last_mouse_y = p_ypos;
}

// ----------------------------------------------------------------------------
void FlyController::handleMouseButton(int p_button,
                                      int p_action,
                                      int /*p_mods*/)
{
    if (p_button == GLFW_MOUSE_BUTTON_RIGHT)
    {
        if (p_action == GLFW_PRESS)
        {
            m_is_looking = true;
            m_first_mouse = true;
        }
        else if (p_action == GLFW_RELEASE)
        {
            m_is_looking = false;
        }
    }
}

// ----------------------------------------------------------------------------
void FlyController::handleScroll(double /*p_xoffset*/, double p_yoffset)
{
    // Adjust movement speed with scroll
    m_move_speed += static_cast<float>(p_yoffset) * 0.5f;
    if (m_move_speed < 0.1f)
        m_move_speed = 0.1f;
    if (m_move_speed > 50.0f)
        m_move_speed = 50.0f;
}

// ----------------------------------------------------------------------------
void FlyController::handleKeyboard(int p_key, int p_action, int p_mods)
{
    bool is_pressed = (p_action == GLFW_PRESS || p_action == GLFW_REPEAT);

    switch (p_key)
    {
        case GLFW_KEY_W:
            m_move_forward = is_pressed;
            break;
        case GLFW_KEY_S:
            m_move_backward = is_pressed;
            break;
        case GLFW_KEY_A:
            m_move_left = is_pressed;
            break;
        case GLFW_KEY_D:
            m_move_right = is_pressed;
            break;
        case GLFW_KEY_E:
        case GLFW_KEY_SPACE:
            m_move_up = is_pressed;
            break;
        case GLFW_KEY_Q:
        case GLFW_KEY_LEFT_CONTROL:
            m_move_down = is_pressed;
            break;
        case GLFW_KEY_LEFT_SHIFT:
        case GLFW_KEY_RIGHT_SHIFT:
            m_fast_mode = is_pressed;
            break;
    }
}

// ----------------------------------------------------------------------------
void FlyController::reset()
{
    m_yaw = 0.0f;
    m_pitch = 0.0f;
    m_move_forward = false;
    m_move_backward = false;
    m_move_left = false;
    m_move_right = false;
    m_move_up = false;
    m_move_down = false;
    m_fast_mode = false;
    m_is_looking = false;
    m_first_mouse = true;

    updateCameraOrientation();
}

// ----------------------------------------------------------------------------
void FlyController::updateCameraOrientation()
{
    // Convert yaw and pitch to direction vector
    float yaw_rad = m_yaw * M_PI / 180.0f;
    float pitch_rad = m_pitch * M_PI / 180.0f;

    Eigen::Vector3f direction;
    direction.x() = std::cos(pitch_rad) * std::cos(yaw_rad);
    direction.y() = std::cos(pitch_rad) * std::sin(yaw_rad);
    direction.z() = std::sin(pitch_rad);

    // Update camera target
    m_camera.setTarget(m_camera.position() + direction);
}

} // namespace robotik::renderer
