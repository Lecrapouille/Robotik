/**
 * @file DragController.hpp
 * @brief Drag camera controller for 2D-like navigation.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#pragma once

#include "Robotik/Renderer/Camera/Camera.hpp"
#include "Robotik/Renderer/Camera/CameraController.hpp"

#include <Eigen/Dense>

namespace robotik::renderer
{

// ****************************************************************************
//! \brief Drag camera controller.
//!
//! Simple 2D-like navigation with panning and zooming. Good for top-down
//! views or simple navigation when detailed 3D control is not needed.
//!
//! Controls:
//! - Left/Middle mouse drag: Pan view
//! - Mouse wheel: Zoom in/out
// ****************************************************************************
class DragController: public CameraController
{
public:

    // ------------------------------------------------------------------------
    //! \brief Constructor.
    //! \param p_camera Camera to control.
    //! \param p_target Initial look-at target.
    //! \param p_distance Initial distance from target.
    // ------------------------------------------------------------------------
    explicit DragController(
        Camera& p_camera,
        const Eigen::Vector3f& p_target = Eigen::Vector3f(0.0f, 0.0f, 0.0f),
        float p_distance = 10.0f);

    // ------------------------------------------------------------------------
    //! \brief Set pan sensitivity.
    //! \param p_sensitivity Pan sensitivity.
    // ------------------------------------------------------------------------
    void setPanSensitivity(float p_sensitivity);

    // ------------------------------------------------------------------------
    //! \brief Set zoom sensitivity.
    //! \param p_sensitivity Zoom sensitivity.
    // ------------------------------------------------------------------------
    void setZoomSensitivity(float p_sensitivity);

    // ------------------------------------------------------------------------
    //! \brief Set target point.
    //! \param p_target Target position.
    // ------------------------------------------------------------------------
    void setTarget(const Eigen::Vector3f& p_target) override;

    // ------------------------------------------------------------------------
    //! \brief Update controller state.
    //! \param p_dt Delta time in seconds.
    // ------------------------------------------------------------------------
    void update(float p_dt) override;

    // ------------------------------------------------------------------------
    //! \brief Handle mouse movement.
    //! \param p_xpos Mouse X position.
    //! \param p_ypos Mouse Y position.
    // ------------------------------------------------------------------------
    void handleMouseMove(double p_xpos, double p_ypos) override;

    // ------------------------------------------------------------------------
    //! \brief Handle mouse button events.
    //! \param p_button Mouse button.
    //! \param p_action Action (press/release).
    //! \param p_mods Modifier keys.
    // ------------------------------------------------------------------------
    void handleMouseButton(int p_button, int p_action, int p_mods) override;

    // ------------------------------------------------------------------------
    //! \brief Handle mouse scroll events.
    //! \param p_xoffset Horizontal scroll offset.
    //! \param p_yoffset Vertical scroll offset.
    // ------------------------------------------------------------------------
    void handleScroll(double p_xoffset, double p_yoffset) override;

    // ------------------------------------------------------------------------
    //! \brief Handle keyboard input.
    //! \param p_key Key code.
    //! \param p_action Action.
    //! \param p_mods Modifier keys.
    // ------------------------------------------------------------------------
    void handleKeyboard(int p_key, int p_action, int p_mods) override;

    // ------------------------------------------------------------------------
    //! \brief Reset to default state.
    // ------------------------------------------------------------------------
    void reset() override;

private:

    // ------------------------------------------------------------------------
    //! \brief Update camera position.
    // ------------------------------------------------------------------------
    void updateCameraPosition();

private:

    Eigen::Vector3f m_target; //!< Look-at target
    float m_distance;         //!< Distance from target
    Eigen::Vector3f
        m_direction;          //!< Direction from target to camera (normalized)
    float m_pan_sensitivity;  //!< Pan sensitivity
    float m_zoom_sensitivity; //!< Zoom sensitivity
    float m_min_distance;     //!< Minimum zoom distance
    float m_max_distance;     //!< Maximum zoom distance

    // Mouse state
    bool m_is_dragging;
    double m_last_mouse_x;
    double m_last_mouse_y;
};

} // namespace robotik::renderer
