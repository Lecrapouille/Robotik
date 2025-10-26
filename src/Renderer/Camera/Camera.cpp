/**
 * @file Camera.cpp
 * @brief Abstract base class for 3D cameras implementation.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#include "Robotik/Renderer/Camera/Camera.hpp"

namespace robotik::renderer
{

// ----------------------------------------------------------------------------
Camera::Camera()
    : m_position(0.0f, 0.0f, 5.0f),
      m_target(0.0f, 0.0f, 0.0f),
      m_up(0.0f, 1.0f, 0.0f),
      m_aspect_ratio(1.0f),
      m_view_dirty(true)
{
    m_view_matrix = Eigen::Matrix4f::Identity();
}

// ----------------------------------------------------------------------------
void Camera::setPosition(const Eigen::Vector3f& p_position)
{
    m_position = p_position;
    m_view_dirty = true;
}

// ----------------------------------------------------------------------------
void Camera::setTarget(const Eigen::Vector3f& p_target)
{
    m_target = p_target;
    m_view_dirty = true;
}

// ----------------------------------------------------------------------------
void Camera::setUp(const Eigen::Vector3f& p_up)
{
    m_up = p_up.normalized();
    m_view_dirty = true;
}

// ----------------------------------------------------------------------------
void Camera::lookAt(const Eigen::Vector3f& p_target)
{
    m_target = p_target;
    m_view_dirty = true;
}

// ----------------------------------------------------------------------------
void Camera::lookAt(const Eigen::Vector3f& p_position,
                    const Eigen::Vector3f& p_target,
                    const Eigen::Vector3f& p_up)
{
    m_position = p_position;
    m_target = p_target;
    m_up = p_up.normalized();
    m_view_dirty = true;
}

// ----------------------------------------------------------------------------
void Camera::setAspectRatio(float p_aspect_ratio)
{
    m_aspect_ratio = p_aspect_ratio;
}

// ----------------------------------------------------------------------------
void Camera::update(float /*p_dt*/)
{
    if (m_view_dirty)
    {
        updateViewMatrix();
        m_view_dirty = false;
    }
}

// ----------------------------------------------------------------------------
Eigen::Vector3f Camera::forward() const
{
    return (m_target - m_position).normalized();
}

// ----------------------------------------------------------------------------
Eigen::Vector3f Camera::right() const
{
    return forward().cross(m_up).normalized();
}

// ----------------------------------------------------------------------------
void Camera::updateViewMatrix()
{
    // Build look-at view matrix
    Eigen::Vector3f f = forward();
    Eigen::Vector3f r = right();
    Eigen::Vector3f u = r.cross(f);

    // View matrix transforms world space to camera space
    m_view_matrix << r.x(), r.y(), r.z(), -r.dot(m_position), u.x(), u.y(),
        u.z(), -u.dot(m_position), -f.x(), -f.y(), -f.z(), f.dot(m_position),
        0.0f, 0.0f, 0.0f, 1.0f;
}

} // namespace robotik::renderer
