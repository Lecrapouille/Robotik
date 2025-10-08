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
void Camera::updateMatrices()
{
    // View matrix (look at)
    Eigen::Vector3f forward = (m_camera_target - m_camera_pos).normalized();
    Eigen::Vector3f right = forward.cross(m_camera_up).normalized();
    Eigen::Vector3f up = right.cross(forward);

    m_view_matrix = Eigen::Matrix4f::Identity();
    m_view_matrix(0, 0) = right.x();
    m_view_matrix(1, 0) = right.y();
    m_view_matrix(2, 0) = right.z();
    m_view_matrix(0, 1) = up.x();
    m_view_matrix(1, 1) = up.y();
    m_view_matrix(2, 1) = up.z();
    m_view_matrix(0, 2) = -forward.x();
    m_view_matrix(1, 2) = -forward.y();
    m_view_matrix(2, 2) = -forward.z();
    m_view_matrix(0, 3) = -right.dot(m_camera_pos);
    m_view_matrix(1, 3) = -up.dot(m_camera_pos);
    m_view_matrix(2, 3) = forward.dot(m_camera_pos);

    // Projection matrix (perspective)
    float f = 1.0f / std::tan(m_fov * static_cast<float>(M_PI) / 360.0f);
    m_projection_matrix = Eigen::Matrix4f::Zero();
    m_projection_matrix(0, 0) = f / m_aspect_ratio;
    m_projection_matrix(1, 1) = f;
    m_projection_matrix(2, 2) =
        (m_far_plane + m_near_plane) / (m_near_plane - m_far_plane);
    m_projection_matrix(2, 3) =
        (2.0f * m_far_plane * m_near_plane) / (m_near_plane - m_far_plane);
    m_projection_matrix(3, 2) = -1.0f;
}

// ----------------------------------------------------------------------------
void Camera::setupView(ViewType p_view_type,
                       const Eigen::Vector3f& p_camera_target)
{
    switch (p_view_type)
    {
        case ViewType::TOP:
            m_camera_pos = p_camera_target + Eigen::Vector3f(0.0f, 10.0f, 0.0f);
            m_camera_up = Eigen::Vector3f(0.0f, 0.0f, -1.0f);
            break;
        case ViewType::FRONT:
            m_camera_pos = p_camera_target + Eigen::Vector3f(0.0f, 0.0f, 10.0f);
            m_camera_up = Eigen::Vector3f(0.0f, 1.0f, 0.0f);
            break;
        case ViewType::SIDE:
            m_camera_pos = p_camera_target + Eigen::Vector3f(10.0f, 0.0f, 0.0f);
            m_camera_up = Eigen::Vector3f(0.0f, 1.0f, 0.0f);
            break;
        case ViewType::ISOMETRIC:
            m_camera_pos = p_camera_target + Eigen::Vector3f(7.0f, 7.0f, 7.0f);
            m_camera_up = Eigen::Vector3f(0.0f, 1.0f, 0.0f);
            break;
        case ViewType::PERSPECTIVE:
        default:
            m_camera_pos = p_camera_target + Eigen::Vector3f(8.0f, 3.0f, 8.0f);
            m_camera_up = Eigen::Vector3f(0.0f, 1.0f, 0.0f);
            break;
    }
}

} // namespace robotik::viewer