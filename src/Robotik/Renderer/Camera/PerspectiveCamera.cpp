/**
 * @file PerspectiveCamera.cpp
 * @brief Perspective projection camera implementation.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#include "Robotik/Renderer/Camera/PerspectiveCamera.hpp"

#include <cmath>

namespace robotik::renderer
{

// ----------------------------------------------------------------------------
PerspectiveCamera::PerspectiveCamera(float p_fov,
                                     float p_aspect_ratio,
                                     float p_near,
                                     float p_far)
    : m_fov(p_fov), m_near(p_near), m_far(p_far), m_projection_dirty(true)
{
    m_aspect_ratio = p_aspect_ratio;
    m_projection_matrix = Eigen::Matrix4f::Identity();
}

// ----------------------------------------------------------------------------
void PerspectiveCamera::setFov(float p_fov)
{
    m_fov = p_fov;
    m_projection_dirty = true;
}

// ----------------------------------------------------------------------------
void PerspectiveCamera::setNear(float p_near)
{
    m_near = p_near;
    m_projection_dirty = true;
}

// ----------------------------------------------------------------------------
void PerspectiveCamera::setFar(float p_far)
{
    m_far = p_far;
    m_projection_dirty = true;
}

// ----------------------------------------------------------------------------
void PerspectiveCamera::setClippingPlanes(float p_near, float p_far)
{
    m_near = p_near;
    m_far = p_far;
    m_projection_dirty = true;
}

// ----------------------------------------------------------------------------
void PerspectiveCamera::setAspectRatio(float p_aspect_ratio)
{
    Camera::setAspectRatio(p_aspect_ratio);
    m_projection_dirty = true;
}

// ----------------------------------------------------------------------------
const Eigen::Matrix4f& PerspectiveCamera::projectionMatrix() const
{
    if (m_projection_dirty)
    {
        updateProjectionMatrix();
        m_projection_dirty = false;
    }
    return m_projection_matrix;
}

// ----------------------------------------------------------------------------
void PerspectiveCamera::updateProjectionMatrix() const
{
    // Build perspective projection matrix
    // Using standard OpenGL perspective matrix
    float fov_rad = m_fov * M_PIf / 180.0f;
    float f = 1.0f / std::tan(fov_rad / 2.0f);
    float range_inv = 1.0f / (m_near - m_far);

    m_projection_matrix << f / m_aspect_ratio, 0.0f, 0.0f, 0.0f, 0.0f, f, 0.0f,
        0.0f, 0.0f, 0.0f, (m_near + m_far) * range_inv,
        2.0f * m_near * m_far * range_inv, 0.0f, 0.0f, -1.0f, 0.0f;
}

} // namespace robotik::renderer
