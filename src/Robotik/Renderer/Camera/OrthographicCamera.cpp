/**
 * @file OrthographicCamera.cpp
 * @brief Orthographic projection camera implementation.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#include "Robotik/Renderer/Camera/OrthographicCamera.hpp"

namespace robotik::renderer
{

// ----------------------------------------------------------------------------
OrthographicCamera::OrthographicCamera(float p_size,
                                       float p_aspect_ratio,
                                       float p_near,
                                       float p_far)
    : m_near(p_near), m_far(p_far), m_size(p_size), m_projection_dirty(true)
{
    m_aspect_ratio = p_aspect_ratio;
    updateBoundsFromSize();
    m_projection_matrix = Eigen::Matrix4f::Identity();
}

// ----------------------------------------------------------------------------
OrthographicCamera::OrthographicCamera(float p_left,
                                       float p_right,
                                       float p_bottom,
                                       float p_top,
                                       float p_near,
                                       float p_far)
    : m_left(p_left),
      m_right(p_right),
      m_bottom(p_bottom),
      m_top(p_top),
      m_near(p_near),
      m_far(p_far),
      m_size(0.0f), // Not used in explicit mode
      m_projection_dirty(true)
{
    m_projection_matrix = Eigen::Matrix4f::Identity();
}

// ----------------------------------------------------------------------------
void OrthographicCamera::setSize(float p_size)
{
    m_size = p_size;
    updateBoundsFromSize();
    m_projection_dirty = true;
}

// ----------------------------------------------------------------------------
void OrthographicCamera::setBounds(float p_left,
                                   float p_right,
                                   float p_bottom,
                                   float p_top)
{
    m_left = p_left;
    m_right = p_right;
    m_bottom = p_bottom;
    m_top = p_top;
    m_projection_dirty = true;
}

// ----------------------------------------------------------------------------
void OrthographicCamera::setClippingPlanes(float p_near, float p_far)
{
    m_near = p_near;
    m_far = p_far;
    m_projection_dirty = true;
}

// ----------------------------------------------------------------------------
void OrthographicCamera::setAspectRatio(float p_aspect_ratio)
{
    Camera::setAspectRatio(p_aspect_ratio);
    if (m_size > 0.0f)
    {
        updateBoundsFromSize();
    }
    m_projection_dirty = true;
}

// ----------------------------------------------------------------------------
const Eigen::Matrix4f& OrthographicCamera::projectionMatrix() const
{
    if (m_projection_dirty)
    {
        updateProjectionMatrix();
        m_projection_dirty = false;
    }
    return m_projection_matrix;
}

// ----------------------------------------------------------------------------
void OrthographicCamera::zoom(float p_delta)
{
    m_size -= p_delta;
    if (m_size < 0.1f)
        m_size = 0.1f;
    if (m_size > 100.0f)
        m_size = 100.0f;

    updateBoundsFromSize();
    m_projection_dirty = true;
}

// ----------------------------------------------------------------------------
void OrthographicCamera::updateProjectionMatrix() const
{
    // Build orthographic projection matrix
    float rl = m_right - m_left;
    float tb = m_top - m_bottom;
    float fn = m_far - m_near;

    m_projection_matrix << 2.0f / rl, 0.0f, 0.0f, -(m_right + m_left) / rl,
        0.0f, 2.0f / tb, 0.0f, -(m_top + m_bottom) / tb, 0.0f, 0.0f, -2.0f / fn,
        -(m_far + m_near) / fn, 0.0f, 0.0f, 0.0f, 1.0f;
}

// ----------------------------------------------------------------------------
void OrthographicCamera::updateBoundsFromSize()
{
    m_right = m_size * m_aspect_ratio;
    m_left = -m_right;
    m_top = m_size;
    m_bottom = -m_top;
}

} // namespace robotik::renderer
