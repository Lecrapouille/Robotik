#pragma once

#include <Eigen/Dense>

namespace robotik::viewer
{

// ****************************************************************************
//! \brief Camera management for 3D viewing.
//!
//! Handles camera positioning, view matrices, and projection matrices.
// ****************************************************************************
class Camera
{
public:

    // ------------------------------------------------------------------------
    //! \brief Camera view positions.
    // ------------------------------------------------------------------------
    enum class ViewType
    {
        PERSPECTIVE, //!< Default perspective view
        TOP,         //!< Top-down view
        FRONT,       //!< Front view
        SIDE,        //!< Side view
        ISOMETRIC    //!< Isometric view
    };

    // ------------------------------------------------------------------------
    //! \brief Constructor.
    //! \param p_width Window width.
    //! \param p_height Window height.
    // ------------------------------------------------------------------------
    Camera(size_t p_width, size_t p_height);

    // ------------------------------------------------------------------------
    //! \brief Set camera view.
    //! \param p_camera_target Camera target position.
    // ------------------------------------------------------------------------
    void setView(Eigen::Vector3f const& p_camera_target);

    // ------------------------------------------------------------------------
    //! \brief Set camera view.
    //! \param p_view_type Camera view position.
    // ------------------------------------------------------------------------
    void setView(ViewType p_view_type);

    // ------------------------------------------------------------------------
    //! \brief Update aspect ratio (when window is resized).
    // ------------------------------------------------------------------------
    void setAspectRatio(size_t p_width, size_t p_height);

    // ------------------------------------------------------------------------
    //! \brief Adjust zoom distance.
    //! \param p_delta Zoom delta (positive = zoom in, negative = zoom out).
    // ------------------------------------------------------------------------
    void zoom(float p_delta);

    // ------------------------------------------------------------------------
    //! \brief Get current view type.
    // ------------------------------------------------------------------------
    ViewType getViewType() const
    {
        return m_view_type;
    }

    // ------------------------------------------------------------------------
    //! \brief Get view matrix.
    // ------------------------------------------------------------------------
    Eigen::Matrix4f const& getViewMatrix() const
    {
        return m_view_matrix;
    }

    // ------------------------------------------------------------------------
    //! \brief Get projection matrix.
    // ------------------------------------------------------------------------
    Eigen::Matrix4f const& getProjectionMatrix() const
    {
        return m_projection_matrix;
    }

private:

    // ------------------------------------------------------------------------
    //! \brief Setup camera for specific view.
    // ------------------------------------------------------------------------
    void setupView(ViewType p_view_type,
                   const Eigen::Vector3f& p_camera_target);

    // ------------------------------------------------------------------------
    //! \brief Update camera matrices.
    // ------------------------------------------------------------------------
    void updateMatrices();

    // ------------------------------------------------------------------------
    //! \brief Compute look-at view matrix.
    // ------------------------------------------------------------------------
    void computeLookAtMatrix();

    // ------------------------------------------------------------------------
    //! \brief Compute orthographic projection matrix.
    // ------------------------------------------------------------------------
    void computeOrthographicMatrix();

    // ------------------------------------------------------------------------
    //! \brief Compute perspective projection matrix.
    // ------------------------------------------------------------------------
    void computePerspectiveMatrix();

private:

    ViewType m_view_type = ViewType::PERSPECTIVE;
    Eigen::Vector3f m_camera_pos = Eigen::Vector3f(8.0f, 3.0f, 8.0f);
    Eigen::Vector3f m_camera_target = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
    //! Camera up vector: Use URDF standard (Z-up) not OpenGL standard (Y-up)
    Eigen::Vector3f m_camera_up = Eigen::Vector3f(0.0f, 1.0f, 0.0f);
    float m_fov = 45.0f;
    float m_aspect_ratio = 1.0f;
    float m_near_plane = 0.1f;
    float m_far_plane = 100.0f;
    float m_zoom_distance = 5.0f;
    float m_min_zoom_distance = 0.5f;
    float m_max_zoom_distance = 50.0f;
    Eigen::Matrix4f m_view_matrix;
    Eigen::Matrix4f m_projection_matrix;
};

} // namespace robotik::viewer