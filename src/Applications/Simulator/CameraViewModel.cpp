/**
 * @file CameraViewModel.cpp
 * @brief Camera view model implementation.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#include "CameraViewModel.hpp"

namespace robotik::application
{

// ----------------------------------------------------------------------------
CameraViewModel::CameraViewModel(size_t p_window_width, size_t p_window_height)
{
    float aspect_ratio = static_cast<float>(p_window_width) /
                         static_cast<float>(p_window_height);

    // Create the perspective camera
    m_camera = std::make_unique<renderer::PerspectiveCamera>(
        45.0f, aspect_ratio, 0.1f, 100.0f);
    m_camera->setPosition(Eigen::Vector3f(3.0f, 3.0f, 3.0f));
    m_camera->lookAt(Eigen::Vector3f(0.0f, 0.0f, 0.0f));

    // Create two camera controllers
    Eigen::Vector3f initial_target(0.0f, 0.0f, 0.5f);

    m_orbit_controller = std::make_unique<renderer::OrbitController>(
        *m_camera, initial_target, m_default_distance);

    m_drag_controller = std::make_unique<renderer::DragController>(
        *m_camera, initial_target, m_default_distance);

    // Start with OrbitController
    m_current_controller = m_orbit_controller.get();
}

// ----------------------------------------------------------------------------
void CameraViewModel::setView(ViewType p_view_type)
{
    Eigen::Vector3f target = getCurrentTarget();

    // For all other views, calculate position and up vector
    Eigen::Vector3f position;
    Eigen::Vector3f up;

    switch (p_view_type)
    {
        case ViewType::TOP:
            // Looking down from Z+ (top view)
            position = target + Eigen::Vector3f(0.0f, 0.0f, m_default_distance);
            up = Eigen::Vector3f(0.0f, 1.0f, 0.0f);
            break;

        case ViewType::BOTTOM:
            // Looking up from Z- (bottom view)
            position =
                target + Eigen::Vector3f(0.0f, 0.0f, -m_default_distance);
            up = Eigen::Vector3f(0.0f, 1.0f, 0.0f);
            break;

        case ViewType::FRONT:
            // Looking from Y+ (front view)
            position = target + Eigen::Vector3f(0.0f, m_default_distance, 0.0f);
            up = Eigen::Vector3f(0.0f, 0.0f, 1.0f);
            break;

        case ViewType::BACK:
            // Looking from Y- (back view)
            position =
                target + Eigen::Vector3f(0.0f, -m_default_distance, 0.0f);
            up = Eigen::Vector3f(0.0f, 0.0f, 1.0f);
            break;

        case ViewType::RIGHT:
            // Looking from X+ (right view)
            position = target + Eigen::Vector3f(m_default_distance, 0.0f, 0.0f);
            up = Eigen::Vector3f(0.0f, 0.0f, 1.0f);
            break;

        case ViewType::LEFT:
            // Looking from X- (left view)
            position =
                target + Eigen::Vector3f(-m_default_distance, 0.0f, 0.0f);
            up = Eigen::Vector3f(0.0f, 0.0f, 1.0f);
            break;

        case ViewType::ORBIT:
            // Recreate orbit controller with current target
            m_orbit_controller = std::make_unique<renderer::OrbitController>(
                *m_camera, target, m_default_distance);
            m_current_controller = m_orbit_controller.get();
            return;
    }

    // Update camera
    m_camera->lookAt(position, target, up);

    // Create drag controller to preserve this view
    Eigen::Vector3f camera_position = m_camera->position();
    Eigen::Vector3f direction = target - camera_position;
    float distance = direction.norm();

    // If distance is too small, use default values
    if (distance < 0.1f)
    {
        distance = m_default_distance;
    }

    m_drag_controller =
        std::make_unique<renderer::DragController>(*m_camera, target, distance);
    m_current_controller = m_drag_controller.get();
}

// ----------------------------------------------------------------------------
void CameraViewModel::setTarget(Eigen::Vector3f const& p_target)
{
    if (m_current_controller)
    {
        m_current_controller->setTarget(p_target);
    }
}

// ----------------------------------------------------------------------------
void CameraViewModel::setTrackingEnabled(bool p_enabled)
{
    m_tracking_enabled = p_enabled;
}

// ----------------------------------------------------------------------------
void CameraViewModel::update(float p_dt,
                             Eigen::Vector3f const* p_tracking_target)
{
    // Update camera controller
    if (m_current_controller)
    {
        m_current_controller->update(p_dt);
    }

    // Apply tracking if enabled and target is provided
    if (m_tracking_enabled && p_tracking_target != nullptr &&
        !isUserInteracting())
    {
        setTarget(*p_tracking_target);
    }
}

// ----------------------------------------------------------------------------
void CameraViewModel::handleMouseButton(int p_button, int p_action, int p_mods)
{
    // Block user interaction if tracking is enabled
    if (!isUserInteractionAllowed())
        return;

    if (m_current_controller)
    {
        m_current_controller->handleMouseButton(p_button, p_action, p_mods);
    }
}

// ----------------------------------------------------------------------------
void CameraViewModel::handleMouseMove(double p_xpos, double p_ypos)
{
    // Block user interaction if tracking is enabled
    if (!isUserInteractionAllowed())
        return;

    if (m_current_controller)
    {
        m_current_controller->handleMouseMove(p_xpos, p_ypos);
    }
}

// ----------------------------------------------------------------------------
void CameraViewModel::handleScroll(double p_xoffset, double p_yoffset)
{
    // Block user interaction if tracking is enabled
    if (!isUserInteractionAllowed())
        return;

    if (m_current_controller)
    {
        m_current_controller->handleScroll(p_xoffset, p_yoffset);
    }
}

// ----------------------------------------------------------------------------
void CameraViewModel::onWindowResize(int p_width, int p_height)
{
    auto aspect_ratio =
        static_cast<float>(p_width) / static_cast<float>(p_height);
    m_camera->setAspectRatio(aspect_ratio);
}

// ----------------------------------------------------------------------------
bool CameraViewModel::isUserInteractionAllowed() const
{
    // Don't allow user interaction when tracking is enabled
    return !m_tracking_enabled;
}

// ----------------------------------------------------------------------------
bool CameraViewModel::isUserInteracting() const
{
    // Check if user is currently interacting with orbit controller
    if (m_orbit_controller && m_current_controller == m_orbit_controller.get())
    {
        return m_orbit_controller->isUserInteracting();
    }

    // Drag controller doesn't have isUserInteracting, so return false
    return false;
}

// ----------------------------------------------------------------------------
Eigen::Vector3f CameraViewModel::getCurrentTarget() const
{
    return m_camera->target();
}

} // namespace robotik::application
