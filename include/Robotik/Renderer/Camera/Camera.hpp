/**
 * @file Camera.hpp
 * @brief Abstract base class for 3D cameras.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#pragma once

#include <Eigen/Dense>

namespace robotik::renderer
{

// ****************************************************************************
//! \brief Abstract base class for 3D cameras.
//!
//! Provides common camera functionality for position, orientation, and view
//! matrix computation. Derived classes implement specific projection types
//! (perspective, orthographic).
// ****************************************************************************
class Camera
{
public:

    // ------------------------------------------------------------------------
    //! \brief Virtual destructor.
    // ------------------------------------------------------------------------
    virtual ~Camera() = default;

    // ------------------------------------------------------------------------
    //! \brief Set camera position in world space.
    //! \param p_position Camera position.
    // ------------------------------------------------------------------------
    void setPosition(const Eigen::Vector3f& p_position);

    // ------------------------------------------------------------------------
    //! \brief Set camera target (look-at point).
    //! \param p_target Target position in world space.
    // ------------------------------------------------------------------------
    void setTarget(const Eigen::Vector3f& p_target);

    // ------------------------------------------------------------------------
    //! \brief Set camera up vector.
    //! \param p_up Up vector (default: Y-up for OpenGL, but Z-up for URDF).
    // ------------------------------------------------------------------------
    void setUp(const Eigen::Vector3f& p_up);

    // ------------------------------------------------------------------------
    //! \brief Set camera to look at a specific point.
    //! \param p_target Target position in world space.
    // ------------------------------------------------------------------------
    void lookAt(const Eigen::Vector3f& p_target);

    // ------------------------------------------------------------------------
    //! \brief Set camera to look at a point from a position.
    //! \param p_position Camera position.
    //! \param p_target Target position.
    //! \param p_up Up vector.
    // ------------------------------------------------------------------------
    void lookAt(const Eigen::Vector3f& p_position,
                const Eigen::Vector3f& p_target,
                const Eigen::Vector3f& p_up);

    // ------------------------------------------------------------------------
    //! \brief Set aspect ratio (width/height).
    //! \param p_aspect_ratio Aspect ratio.
    // ------------------------------------------------------------------------
    virtual void setAspectRatio(float p_aspect_ratio);

    // ------------------------------------------------------------------------
    //! \brief Update camera state (called each frame).
    //! \param p_dt Delta time in seconds.
    // ------------------------------------------------------------------------
    virtual void update(float p_dt);

    // ------------------------------------------------------------------------
    //! \brief Get view matrix (world to camera space).
    //! \return View matrix.
    // ------------------------------------------------------------------------
    const Eigen::Matrix4f& viewMatrix() const
    {
        return m_view_matrix;
    }

    // ------------------------------------------------------------------------
    //! \brief Get projection matrix (camera to clip space).
    //! \return Projection matrix.
    // ------------------------------------------------------------------------
    virtual const Eigen::Matrix4f& projectionMatrix() const = 0;

    // ------------------------------------------------------------------------
    //! \brief Get camera position.
    //! \return Camera position in world space.
    // ------------------------------------------------------------------------
    const Eigen::Vector3f& position() const
    {
        return m_position;
    }

    // ------------------------------------------------------------------------
    //! \brief Get camera target.
    //! \return Target position in world space.
    // ------------------------------------------------------------------------
    const Eigen::Vector3f& target() const
    {
        return m_target;
    }

    // ------------------------------------------------------------------------
    //! \brief Get camera up vector.
    //! \return Up vector.
    // ------------------------------------------------------------------------
    const Eigen::Vector3f& up() const
    {
        return m_up;
    }

    // ------------------------------------------------------------------------
    //! \brief Get forward direction vector.
    //! \return Normalized forward vector.
    // ------------------------------------------------------------------------
    Eigen::Vector3f forward() const;

    // ------------------------------------------------------------------------
    //! \brief Get right direction vector.
    //! \return Normalized right vector.
    // ------------------------------------------------------------------------
    Eigen::Vector3f right() const;

protected:

    // ------------------------------------------------------------------------
    //! \brief Constructor (protected - only derived classes can instantiate).
    // ------------------------------------------------------------------------
    Camera();

    // ------------------------------------------------------------------------
    //! \brief Update view matrix from position, target, and up.
    // ------------------------------------------------------------------------
    void updateViewMatrix();

protected:

    Eigen::Vector3f m_position;    //!< Camera position
    Eigen::Vector3f m_target;      //!< Look-at target
    Eigen::Vector3f m_up;          //!< Up vector
    float m_aspect_ratio;          //!< Aspect ratio (width/height)
    Eigen::Matrix4f m_view_matrix; //!< View matrix (cached)
    bool m_view_dirty;             //!< View matrix needs update
};

} // namespace robotik::renderer
