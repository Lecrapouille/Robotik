#pragma once

// #include "Robotik/Renderer/Camera.hpp"

#include <string>
#include <vector>

#include <Eigen/Dense>

namespace robotik::application
{

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
    std::vector<std::string> urdf_files;
    //! Joint positions to set
    std::vector<double> joint_positions;
    //! Control joint for inverse kinematics: usually the tool center point
    std::string control_joint;
    //! Camera target joint: usually the base link or the tool center point
    std::string camera_target;
    //! Show frames
    bool show_frames = false;
    //! Frame scale
    float frame_scale = 0.1f;
    //! Physics gravity
    Eigen::Vector3d physics_gravity = Eigen::Vector3d(0.0, 0.0, -9.81);
};

} // namespace robotik::application