/**
 * @file OrthographicCamera.hpp
 * @brief Orthographic projection camera.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#pragma once

#include "Robotik/Renderer/Camera/Camera.hpp"

namespace robotik::renderer
{

// ****************************************************************************
//! \brief Orthographic projection camera.
//!
//! Implements an orthographic (parallel) projection. Useful for technical
//! views (TOP, FRONT, SIDE) where perspective distortion is not desired.
//! Essential for robotics visualization and CAD-like views.
// ****************************************************************************
class OrthographicCamera: public Camera
{
public:

    // ------------------------------------------------------------------------
    //! \brief Constructor with symmetric bounds.
    //! \param p_size Half-size of the view (symmetric around center).
    //! \param p_aspect_ratio Aspect ratio (width/height).
    //! \param p_near Near clipping plane distance.
    //! \param p_far Far clipping plane distance.
    // ------------------------------------------------------------------------
    explicit OrthographicCamera(float p_size = 5.0f,
                                float p_aspect_ratio = 1.0f,
                                float p_near = 0.1f,
                                float p_far = 100.0f);

    // ------------------------------------------------------------------------
    //! \brief Constructor with explicit bounds.
    //! \param p_left Left bound.
    //! \param p_right Right bound.
    //! \param p_bottom Bottom bound.
    //! \param p_top Top bound.
    //! \param p_near Near clipping plane distance.
    //! \param p_far Far clipping plane distance.
    // ------------------------------------------------------------------------
    OrthographicCamera(float p_left,
                       float p_right,
                       float p_bottom,
                       float p_top,
                       float p_near,
                       float p_far);

    // ------------------------------------------------------------------------
    //! \brief Set orthographic size (half-size, symmetric).
    //! \param p_size Half-size of the view.
    // ------------------------------------------------------------------------
    void setSize(float p_size);

    // ------------------------------------------------------------------------
    //! \brief Set orthographic bounds explicitly.
    //! \param p_left Left bound.
    //! \param p_right Right bound.
    //! \param p_bottom Bottom bound.
    //! \param p_top Top bound.
    // ------------------------------------------------------------------------
    void setBounds(float p_left, float p_right, float p_bottom, float p_top);

    // ------------------------------------------------------------------------
    //! \brief Set clipping planes.
    //! \param p_near Near plane distance.
    //! \param p_far Far plane distance.
    // ------------------------------------------------------------------------
    void setClippingPlanes(float p_near, float p_far);

    // ------------------------------------------------------------------------
    //! \brief Set aspect ratio and update bounds.
    //! \param p_aspect_ratio Aspect ratio.
    // ------------------------------------------------------------------------
    void setAspectRatio(float p_aspect_ratio) override;

    // ------------------------------------------------------------------------
    //! \brief Get projection matrix.
    //! \return Projection matrix.
    // ------------------------------------------------------------------------
    const Eigen::Matrix4f& projectionMatrix() const override;

    // ------------------------------------------------------------------------
    //! \brief Zoom in/out by changing orthographic size.
    //! \param p_delta Zoom delta (positive = zoom in).
    // ------------------------------------------------------------------------
    void zoom(float p_delta);

private:

    // ------------------------------------------------------------------------
    //! \brief Update projection matrix.
    // ------------------------------------------------------------------------
    void updateProjectionMatrix() const;

    // ------------------------------------------------------------------------
    //! \brief Update bounds from size and aspect ratio.
    // ------------------------------------------------------------------------
    void updateBoundsFromSize();

private:

    float m_left;   //!< Left bound
    float m_right;  //!< Right bound
    float m_bottom; //!< Bottom bound
    float m_top;    //!< Top bound
    float m_near;   //!< Near clipping plane
    float m_far;    //!< Far clipping plane
    float m_size;   //!< Orthographic size (for symmetric mode)
    mutable Eigen::Matrix4f m_projection_matrix; //!< Cached projection matrix
    mutable bool m_projection_dirty;             //!< Projection needs update
};

} // namespace robotik::renderer
