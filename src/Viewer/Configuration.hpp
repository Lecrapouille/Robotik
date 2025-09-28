#pragma once

#include "Viewer/Camera.hpp"

#include <string>

namespace robotik::viewer
{

// ************************************************************************
//! \brief Control modes for the robot.
// ************************************************************************
enum class ControlMode
{
    //! Automatic sinusoidal animation
    ANIMATION,
    //! Interactive inverse kinematics control
    INVERSE_KINEMATICS
};

// ************************************************************************
//! \brief Configuration for the application.
// ************************************************************************
struct Configuration
{
    //! Window width in pixels
    size_t window_width = 1024;
    //! Window height in pixels
    size_t window_height = 768;
    //! Window title
    std::string window_title = "Robot Viewer";
    //! Target frame rate in FPS
    size_t target_fps = 60;
    //! Target physics update rate in Hz
    size_t target_physics_hz = 15;
    //! Search paths data (STL files ...)
    std::string search_paths;
    //! Path to the URDF file to load
    std::string urdf_file;
    //! Control joint for inverse kinematics: usually the tool center point
    std::string control_joint;
    //! Camera target joint: usually the base link or the tool center point
    std::string camera_target;
    //! Camera view type
    Camera::ViewType camera_view = Camera::ViewType::ISOMETRIC;
    //! Enable performance profiling
    bool enable_profiling = false;
    //! Show frames
    bool show_frames = false;
    //! Frame scale
    float frame_scale = 0.1f;
};

} // namespace robotik::viewer