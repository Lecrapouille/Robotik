/**
 * @file CameraController.hpp
 * @brief Camera view model for managing camera state and interactions.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#pragma once

#include "Robotik/Renderer/Camera/CameraController.hpp"
#include "Robotik/Renderer/Camera/DragController.hpp"
#include "Robotik/Renderer/Camera/OrbitController.hpp"
#include "Robotik/Renderer/Camera/PerspectiveCamera.hpp"

#include <Eigen/Dense>
#include <memory>

namespace robotik::application
{

// ****************************************************************************
//! \brief Camera view model (MVC pattern).
//!
//! This class encapsulates the camera, camera controllers, and the logic for
//! switching between different camera modes and views. It also handles camera
//! tracking with proper user interaction management to avoid glitches.
// ****************************************************************************
class CameraController
{
public:

    // ----------------------------------------------------------------------------
    //! \brief View types for predefined camera positions.
    // ----------------------------------------------------------------------------
    enum class ViewType
    {
        TOP,    //!< Top view (looking down from Z+)
        BOTTOM, //!< Bottom view (looking up from Z-)
        FRONT,  //!< Front view (looking from Y+)
        BACK,   //!< Back view (looking from Y-)
        RIGHT,  //!< Right view (looking from X+)
        LEFT,   //!< Left view (looking from X-)
        ORBIT   //!< Orbit mode (free rotation around target)
    };

    // ----------------------------------------------------------------------------
    //! \brief Constructor.
    //! \param p_window_width Window width for aspect ratio.
    //! \param p_window_height Window height for aspect ratio.
    // ----------------------------------------------------------------------------
    CameraController(size_t p_window_width, size_t p_window_height);

    // ----------------------------------------------------------------------------
    //! \brief Set camera to a predefined view.
    //! \param p_view_type View type to set.
    // ----------------------------------------------------------------------------
    void setView(ViewType p_view_type);

    // ----------------------------------------------------------------------------
    //! \brief Set camera target position.
    //! \param p_target Target position in world space.
    // ----------------------------------------------------------------------------
    void setTarget(Eigen::Vector3f const& p_target);

    // ----------------------------------------------------------------------------
    //! \brief Enable or disable camera tracking.
    //! \param p_enabled True to enable tracking, false to disable.
    // ----------------------------------------------------------------------------
    void setTrackingEnabled(bool p_enabled);

    // ----------------------------------------------------------------------------
    //! \brief Check if camera tracking is enabled.
    //! \return True if tracking is enabled, false otherwise.
    // ----------------------------------------------------------------------------
    bool isTrackingEnabled() const
    {
        return m_tracking_enabled;
    }

    // ----------------------------------------------------------------------------
    //! \brief Get the camera (const access).
    //! \return Const reference to the perspective camera.
    // ----------------------------------------------------------------------------
    renderer::PerspectiveCamera const& camera() const
    {
        return *m_camera;
    }

    // ----------------------------------------------------------------------------
    //! \brief Get the camera (non-const access for rendering).
    //! \return Reference to the perspective camera.
    // ----------------------------------------------------------------------------
    renderer::PerspectiveCamera& camera()
    {
        return *m_camera;
    }

    // ----------------------------------------------------------------------------
    //! \brief Update camera state and tracking.
    //! \param p_dt Delta time in seconds.
    //! \param p_tracking_target Optional target position for tracking.
    // ----------------------------------------------------------------------------
    void update(float p_dt, Eigen::Vector3f const* p_tracking_target = nullptr);

    // ----------------------------------------------------------------------------
    //! \brief Handle mouse button events.
    //! \param p_button Mouse button.
    //! \param p_action Action (press/release).
    //! \param p_mods Modifier keys.
    // ----------------------------------------------------------------------------
    void handleMouseButton(int p_button, int p_action, int p_mods);

    // ----------------------------------------------------------------------------
    //! \brief Handle mouse movement.
    //! \param p_xpos Mouse X position.
    //! \param p_ypos Mouse Y position.
    // ----------------------------------------------------------------------------
    void handleMouseMove(double p_xpos, double p_ypos);

    // ----------------------------------------------------------------------------
    //! \brief Handle mouse scroll events.
    //! \param p_xoffset Horizontal scroll offset.
    //! \param p_yoffset Vertical scroll offset.
    // ----------------------------------------------------------------------------
    void handleScroll(double p_xoffset, double p_yoffset);

    // ----------------------------------------------------------------------------
    //! \brief Handle window resize.
    //! \param p_width New window width.
    //! \param p_height New window height.
    // ----------------------------------------------------------------------------
    void onWindowResize(int p_width, int p_height);

private:

    // ----------------------------------------------------------------------------
    //! \brief Check if user interaction is allowed (not blocked by tracking).
    //! \return True if user can interact with camera, false otherwise.
    // ----------------------------------------------------------------------------
    bool isUserInteractionAllowed() const;

    // ----------------------------------------------------------------------------
    //! \brief Check if user is currently interacting with the camera.
    //! \return True if user is interacting, false otherwise.
    // ----------------------------------------------------------------------------
    bool isUserInteracting() const;

    // ----------------------------------------------------------------------------
    //! \brief Get current camera target position.
    //! \return Target position.
    // ----------------------------------------------------------------------------
    Eigen::Vector3f getCurrentTarget() const;

private:

    //! \brief Perspective camera.
    std::unique_ptr<renderer::PerspectiveCamera> m_camera;
    //! \brief Current camera controller (non-owning pointer).
    renderer::CameraController* m_current_controller = nullptr;
    //! \brief Orbit controller instance.
    std::unique_ptr<renderer::OrbitController> m_orbit_controller;
    //! \brief Drag controller instance.
    std::unique_ptr<renderer::DragController> m_drag_controller;
    //! \brief Whether camera tracking is enabled.
    bool m_tracking_enabled = false;
    //! \brief Default distance from target for predefined views.
    float m_default_distance = 5.0f;
};

} // namespace robotik::application
