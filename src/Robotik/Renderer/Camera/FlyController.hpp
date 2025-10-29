/**
 * @file FlyController.hpp
 * @brief Fly camera controller for free movement.
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
//! \brief Fly camera controller.
//!
//! Free movement in 3D space (FPS-style). Useful for navigating large
//! environments or exploring complex robot workspaces.
//!
//! Controls:
//! - W/S: Move forward/backward
//! - A/D: Move left/right
//! - Q/E: Move down/up
//! - Mouse drag: Look around
//! - Shift: Move faster
// ****************************************************************************
class FlyController: public CameraController
{
public:

    // ------------------------------------------------------------------------
    //! \brief Constructor.
    //! \param p_camera Camera to control.
    //! \param p_move_speed Movement speed (units per second).
    // ------------------------------------------------------------------------
    explicit FlyController(Camera& p_camera, float p_move_speed = 3.0f);

    // ------------------------------------------------------------------------
    //! \brief Set movement speed.
    //! \param p_speed Movement speed.
    // ------------------------------------------------------------------------
    void setMoveSpeed(float p_speed);

    // ------------------------------------------------------------------------
    //! \brief Set look sensitivity.
    //! \param p_sensitivity Look sensitivity.
    // ------------------------------------------------------------------------
    void setLookSensitivity(float p_sensitivity);

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
    //! \brief Update camera orientation from yaw and pitch.
    // ------------------------------------------------------------------------
    void updateCameraOrientation();

private:

    float m_move_speed;       //!< Movement speed
    float m_look_sensitivity; //!< Look sensitivity
    float m_speed_multiplier; //!< Speed multiplier (for shift)
    float m_yaw;              //!< Yaw angle (horizontal rotation)
    float m_pitch;            //!< Pitch angle (vertical rotation)

    // Movement state
    bool m_move_forward;
    bool m_move_backward;
    bool m_move_left;
    bool m_move_right;
    bool m_move_up;
    bool m_move_down;
    bool m_fast_mode;

    // Mouse state
    bool m_is_looking;
    double m_last_mouse_x;
    double m_last_mouse_y;
    bool m_first_mouse;
};

} // namespace robotik::renderer
