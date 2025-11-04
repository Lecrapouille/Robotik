/**
 * @file OrbitController.hpp
 * @brief Orbit camera controller for object inspection.
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
//! \brief Orbit camera controller.
//!
//! Orbits the camera around a fixed target point. Perfect for inspecting
//! objects (like robots) from all angles. This is the primary controller
//! for robotics visualization.
//!
//! Controls:
//! - Left mouse drag: Orbit around target
//! - Right mouse drag: Pan target point
//! - Mouse wheel: Zoom in/out
//! - Middle mouse drag: Pan target point
// ****************************************************************************
class OrbitController: public CameraController
{
public:

    // ------------------------------------------------------------------------
    //! \brief Constructor.
    //! \param p_camera Camera to control.
    //! \param p_target Initial orbit target.
    //! \param p_distance Initial distance from target.
    // ------------------------------------------------------------------------
    explicit OrbitController(
        Camera& p_camera,
        const Eigen::Vector3f& p_target = Eigen::Vector3f(0.0f, 0.0f, 0.0f),
        float p_distance = 5.0f);

    // ------------------------------------------------------------------------
    //! \brief Set orbit target point.
    //! \param p_target Target position in world space.
    // ------------------------------------------------------------------------
    void setTarget(const Eigen::Vector3f& p_target) override;

    // ------------------------------------------------------------------------
    //! \brief Set orbit distance from target.
    //! \param p_distance Distance from target.
    // ------------------------------------------------------------------------
    void setDistance(float p_distance);

    // ------------------------------------------------------------------------
    //! \brief Set orbit sensitivity.
    //! \param p_sensitivity Rotation sensitivity (default: 0.3).
    // ------------------------------------------------------------------------
    void setSensitivity(float p_sensitivity);

    // ------------------------------------------------------------------------
    //! \brief Set zoom sensitivity.
    //! \param p_sensitivity Zoom sensitivity (default: 0.1).
    // ------------------------------------------------------------------------
    void setZoomSensitivity(float p_sensitivity);

    // ------------------------------------------------------------------------
    //! \brief Set pan sensitivity.
    //! \param p_sensitivity Pan sensitivity (default: 0.01).
    // ------------------------------------------------------------------------
    void setPanSensitivity(float p_sensitivity);

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

    // ------------------------------------------------------------------------
    //! \brief Check if user is currently interacting with the camera.
    //! \return True if user is orbiting or panning, false otherwise.
    // ------------------------------------------------------------------------
    bool isUserInteracting() const;

private:

    // ------------------------------------------------------------------------
    //! \brief Update camera position from spherical coordinates.
    // ------------------------------------------------------------------------
    void updateCameraPosition();

private:

    Eigen::Vector3f m_target; //!< Orbit target point
    float m_distance;         //!< Distance from target
    float m_azimuth;          //!< Azimuth angle (horizontal rotation)
    float m_elevation;        //!< Elevation angle (vertical rotation)
    float m_sensitivity;      //!< Rotation sensitivity
    float m_zoom_sensitivity; //!< Zoom sensitivity
    float m_pan_sensitivity;  //!< Pan sensitivity
    float m_min_distance;     //!< Minimum zoom distance
    float m_max_distance;     //!< Maximum zoom distance
    float m_min_elevation;    //!< Minimum elevation angle
    float m_max_elevation;    //!< Maximum elevation angle

    // Mouse state
    bool m_is_orbiting;    //!< Currently orbiting
    bool m_is_panning;     //!< Currently panning
    double m_last_mouse_x; //!< Last mouse X position
    double m_last_mouse_y; //!< Last mouse Y position
};

} // namespace robotik::renderer
