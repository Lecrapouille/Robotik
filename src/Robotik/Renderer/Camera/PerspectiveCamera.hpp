/**
 * @file PerspectiveCamera.hpp
 * @brief Perspective projection camera.
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
//! \brief Perspective projection camera.
//!
//! Implements a perspective projection with field of view, aspect ratio,
//! and near/far clipping planes. Standard camera for 3D rendering.
// ****************************************************************************
class PerspectiveCamera: public Camera
{
public:

    // ------------------------------------------------------------------------
    //! \brief Constructor.
    //! \param p_fov Field of view in degrees (vertical).
    //! \param p_aspect_ratio Aspect ratio (width/height).
    //! \param p_near Near clipping plane distance.
    //! \param p_far Far clipping plane distance.
    // ------------------------------------------------------------------------
    PerspectiveCamera(float p_fov = 45.0f,
                      float p_aspect_ratio = 1.0f,
                      float p_near = 0.1f,
                      float p_far = 100.0f);

    // ------------------------------------------------------------------------
    //! \brief Set field of view.
    //! \param p_fov Field of view in degrees (vertical).
    // ------------------------------------------------------------------------
    void setFov(float p_fov);

    // ------------------------------------------------------------------------
    //! \brief Set near clipping plane.
    //! \param p_near Near plane distance.
    // ------------------------------------------------------------------------
    void setNear(float p_near);

    // ------------------------------------------------------------------------
    //! \brief Set far clipping plane.
    //! \param p_far Far plane distance.
    // ------------------------------------------------------------------------
    void setFar(float p_far);

    // ------------------------------------------------------------------------
    //! \brief Set clipping planes.
    //! \param p_near Near plane distance.
    //! \param p_far Far plane distance.
    // ------------------------------------------------------------------------
    void setClippingPlanes(float p_near, float p_far);

    // ------------------------------------------------------------------------
    //! \brief Set aspect ratio override.
    //! \param p_aspect_ratio Aspect ratio.
    // ------------------------------------------------------------------------
    void setAspectRatio(float p_aspect_ratio) override;

    // ------------------------------------------------------------------------
    //! \brief Get projection matrix.
    //! \return Projection matrix.
    // ------------------------------------------------------------------------
    const Eigen::Matrix4f& projectionMatrix() const override;

    // ------------------------------------------------------------------------
    //! \brief Get field of view.
    //! \return FOV in degrees.
    // ------------------------------------------------------------------------
    float fov() const
    {
        return m_fov;
    }

    // ------------------------------------------------------------------------
    //! \brief Get near plane distance.
    //! \return Near plane distance.
    // ------------------------------------------------------------------------
    float near() const
    {
        return m_near;
    }

    // ------------------------------------------------------------------------
    //! \brief Get far plane distance.
    //! \return Far plane distance.
    // ------------------------------------------------------------------------
    float far() const
    {
        return m_far;
    }

private:

    // ------------------------------------------------------------------------
    //! \brief Update projection matrix.
    // ------------------------------------------------------------------------
    void updateProjectionMatrix() const;

private:

    float m_fov;                                 //!< Field of view (degrees)
    float m_near;                                //!< Near clipping plane
    float m_far;                                 //!< Far clipping plane
    mutable Eigen::Matrix4f m_projection_matrix; //!< Cached projection matrix
    mutable bool m_projection_dirty;             //!< Projection needs update
};

} // namespace robotik::renderer
