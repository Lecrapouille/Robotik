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
    //! Control link for inverse kinematics: usually the tool center point
    std::string control_link;
    //! Camera target joint: usually the base link or the tool center point
    std::string camera_target;
    //! Show joint axes for debugging
    bool show_joint_axes = true;
    //! Show revolute joint axes
    bool show_revolute_joint_axes = true;
    //! Show prismatic joint axes
    bool show_prismatic_joint_axes = true;
    //! Physics gravity
    Eigen::Vector3d physics_gravity = Eigen::Vector3d(0.0, 0.0, -9.81);
    //! Use root if camera target is not found
    bool use_root_if_not_found = true;
};

} // namespace robotik::application