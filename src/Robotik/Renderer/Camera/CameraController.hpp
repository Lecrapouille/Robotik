/**
 * @file CameraController.hpp
 * @brief Interface for camera controllers.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#pragma once

#include <Eigen/Dense>

namespace robotik::renderer
{

// Forward declaration
class Camera;

// ****************************************************************************
//! \brief Interface for camera controllers.
//!
//! Camera controllers handle user input (mouse, keyboard) and update
//! camera state accordingly. Different controllers provide different
//! navigation styles (orbit, fly, drag, etc.).
// ****************************************************************************
class CameraController
{
public:

    // ------------------------------------------------------------------------
    //! \brief Virtual destructor.
    // ------------------------------------------------------------------------
    virtual ~CameraController() = default;

    // ------------------------------------------------------------------------
    //! \brief Update controller state (called each frame).
    //! \param p_dt Delta time in seconds.
    // ------------------------------------------------------------------------
    virtual void update(float p_dt) = 0;

    // ------------------------------------------------------------------------
    //! \brief Handle mouse movement.
    //! \param p_xpos Mouse X position in screen coordinates.
    //! \param p_ypos Mouse Y position in screen coordinates.
    // ------------------------------------------------------------------------
    virtual void handleMouseMove(double p_xpos, double p_ypos) = 0;

    // ------------------------------------------------------------------------
    //! \brief Handle mouse button events.
    //! \param p_button Mouse button (GLFW_MOUSE_BUTTON_*).
    //! \param p_action Action (GLFW_PRESS, GLFW_RELEASE).
    //! \param p_mods Modifier keys (GLFW_MOD_*).
    // ------------------------------------------------------------------------
    virtual void handleMouseButton(int p_button, int p_action, int p_mods) = 0;

    // ------------------------------------------------------------------------
    //! \brief Handle mouse scroll events.
    //! \param p_xoffset Horizontal scroll offset.
    //! \param p_yoffset Vertical scroll offset.
    // ------------------------------------------------------------------------
    virtual void handleScroll(double p_xoffset, double p_yoffset) = 0;

    // ------------------------------------------------------------------------
    //! \brief Handle keyboard input.
    //! \param p_key Key code (GLFW_KEY_*).
    //! \param p_action Action (GLFW_PRESS, GLFW_RELEASE, GLFW_REPEAT).
    //! \param p_mods Modifier keys (GLFW_MOD_*).
    // ------------------------------------------------------------------------
    virtual void handleKeyboard(int p_key, int p_action, int p_mods) = 0;

    // ------------------------------------------------------------------------
    //! \brief Reset controller to default state.
    // ------------------------------------------------------------------------
    virtual void reset() = 0;

    // ------------------------------------------------------------------------
    //! \brief Set target point for camera tracking (optional).
    //! \param p_target Target position in world space.
    //! \note Default implementation does nothing. Override in derived classes
    //! that support target-based navigation (e.g., OrbitController,
    //! DragController).
    // ------------------------------------------------------------------------
    virtual void setTarget(const Eigen::Vector3f& /*p_target*/) {}

protected:

    // ------------------------------------------------------------------------
    //! \brief Constructor (protected - only derived classes can instantiate).
    //! \param p_camera Camera to control.
    // ------------------------------------------------------------------------
    explicit CameraController(Camera& p_camera) : m_camera(p_camera) {}

protected:

    Camera& m_camera; //!< Camera being controlled
};

} // namespace robotik::renderer
