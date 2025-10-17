#include "Robotik/Viewer/Camera.hpp"

namespace robotik::viewer
{

// ----------------------------------------------------------------------------
Camera::Camera(size_t p_width, size_t p_height)
{
    setAspectRatio(p_width, p_height);
}

// ----------------------------------------------------------------------------
void Camera::setView(ViewType p_view_type,
                     const Eigen::Vector3f& p_camera_target)
{
    m_view_type = p_view_type;
    m_camera_target = p_camera_target;
    setupView(p_view_type, m_camera_target);
    updateMatrices();
}

// ----------------------------------------------------------------------------
void Camera::setView(ViewType p_view_type,
                     const Eigen::Vector3d& p_camera_target)
{
    m_view_type = p_view_type;
    m_camera_target = p_camera_target.cast<float>();
    setupView(p_view_type, m_camera_target);
    updateMatrices();
}

// ----------------------------------------------------------------------------
void Camera::setAspectRatio(size_t p_width, size_t p_height)
{
    m_aspect_ratio = static_cast<float>(p_width) / static_cast<float>(p_height);
    updateMatrices();
}

// ----------------------------------------------------------------------------
void Camera::zoom(float p_delta)
{
    m_zoom_distance -= p_delta;

    // Clamp zoom distance to valid range
    if (m_zoom_distance < m_min_zoom_distance)
    {
        m_zoom_distance = m_min_zoom_distance;
    }
    else if (m_zoom_distance > m_max_zoom_distance)
    {
        m_zoom_distance = m_max_zoom_distance;
    }

    setupView(m_view_type, m_camera_target);
    updateMatrices();
}

// ----------------------------------------------------------------------------
void Camera::updateMatrices()
{
    computeLookAtMatrix();

    switch (m_view_type)
    {
        case ViewType::TOP:
        case ViewType::FRONT:
        case ViewType::SIDE:
            computeOrthographicMatrix();
            break;
        case ViewType::PERSPECTIVE:
        case ViewType::ISOMETRIC:
        default:
            computePerspectiveMatrix();
            break;
    }
}

// ----------------------------------------------------------------------------
void Camera::computeLookAtMatrix()
{
    // Build look-at view matrix
    Eigen::Vector3f forward = (m_camera_target - m_camera_pos).normalized();
    Eigen::Vector3f right = forward.cross(m_camera_up).normalized();
    Eigen::Vector3f up = right.cross(forward);

    float tx = -right.dot(m_camera_pos);
    float ty = -up.dot(m_camera_pos);
    float tz = forward.dot(m_camera_pos);

    m_view_matrix << right.x(), up.x(), -forward.x(), tx, right.y(), up.y(),
        -forward.y(), ty, right.z(), up.z(), -forward.z(), tz, 0.0f, 0.0f, 0.0f,
        1.0f;
}

// ----------------------------------------------------------------------------
void Camera::computeOrthographicMatrix()
{
    // Build orthographic projection matrix
    float ortho_size = m_zoom_distance;
    float right = ortho_size * m_aspect_ratio;
    float left = -right;
    float top = ortho_size;
    float bottom = -top;

    float rl = right - left;
    float tb = top - bottom;
    float fn = m_far_plane - m_near_plane;

    m_projection_matrix << 2.0f / rl, 0.0f, 0.0f, -(right + left) / rl, 0.0f,
        2.0f / tb, 0.0f, -(top + bottom) / tb, 0.0f, 0.0f, -2.0f / fn,
        -(m_far_plane + m_near_plane) / fn, 0.0f, 0.0f, 0.0f, 1.0f;
}

// ----------------------------------------------------------------------------
void Camera::computePerspectiveMatrix()
{
    // Build perspective projection matrix
    float f = 1.0f / std::tan(m_fov * static_cast<float>(M_PI) / 360.0f);
    float z_range = (m_far_plane + m_near_plane) / (m_near_plane - m_far_plane);
    float z_mult =
        (2.0f * m_far_plane * m_near_plane) / (m_near_plane - m_far_plane);

    m_projection_matrix << f / m_aspect_ratio, 0.0f, 0.0f, 0.0f, 0.0f, f, 0.0f,
        0.0f, 0.0f, 0.0f, z_range, z_mult, 0.0f, 0.0f, -1.0f, 0.0f;
}

// ----------------------------------------------------------------------------
void Camera::setupView(ViewType p_view_type,
                       const Eigen::Vector3f& p_camera_target)
{
    m_camera_pos = p_camera_target;
    switch (p_view_type)
    {
        case ViewType::PERSPECTIVE:
            m_camera_pos += Eigen::Vector3f(-m_zoom_distance * 0.8f,
                                            m_zoom_distance * 0.8f,
                                            m_zoom_distance * 0.6f);
            m_camera_up = Eigen::Vector3f(0.0f, 0.0f, 1.0f); // Z-up in URDF
            break;
        case ViewType::TOP:
            // Look down toward -Z (down in URDF)
            m_camera_pos += Eigen::Vector3f(0.0f, 0.0f, m_zoom_distance);
            m_camera_up =
                Eigen::Vector3f(1.0f, 0.0f, 0.0f); // X toward screen top
            break;
        case ViewType::FRONT:
            // Look toward +X (forward in URDF)
            m_camera_pos += Eigen::Vector3f(-m_zoom_distance, 0.0f, 0.0f);
            m_camera_up = Eigen::Vector3f(0.0f, 0.0f, 1.0f); // Z up
            break;
        case ViewType::SIDE:
            // Look toward +Y (left in URDF)
            m_camera_pos += Eigen::Vector3f(0.0f, -m_zoom_distance, 0.0f);
            m_camera_up = Eigen::Vector3f(0.0f, 0.0f, 1.0f); // Z up
            break;
        case ViewType::ISOMETRIC:
        default:
        {
            // Isometric view: equal angles with 3 axes
            float iso_angle = 35.264f * static_cast<float>(M_PI) /
                              180.0f; // arctan(1/sqrt(2))
            float azimuth = 45.0f * static_cast<float>(M_PI) / 180.0f;

            float x = m_zoom_distance * std::sin(iso_angle) * std::cos(azimuth);
            float y = m_zoom_distance * std::sin(iso_angle) * std::sin(azimuth);
            float z = m_zoom_distance * std::cos(iso_angle);

            m_camera_pos += Eigen::Vector3f(x, y, z);
            m_camera_up = Eigen::Vector3f(0.0f, 0.0f, 1.0f); // Z up
            break;
        }
    }
}

} // namespace robotik::viewer