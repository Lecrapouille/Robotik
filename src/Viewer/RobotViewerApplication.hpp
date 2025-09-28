/**
 * @file RobotViewerApplication.hpp
 * @brief Robot viewer application class for the 3D viewer.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#pragma once

#include "Viewer/Application.hpp"

#include "Robotik/Robot.hpp"
#include "Robotik/private/Path.hpp"

#include <chrono>
#include <mutex>

namespace robotik
{

// ****************************************************************************
//! \brief Robot viewer application that inherits from Application.
// ****************************************************************************
class RobotViewerApplication: public Application
{
public:

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
        //! Search paths data (STL files ...)
        std::string search_paths;
        //! Path to the URDF file to load
        std::string urdf_file;
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
        //! Control joint for inverse kinematics: usually the tool center point
        std::string control_joint;
        //! Camera target joint: usually the base link or the tool center point
        std::string camera_target;
        //! Camera view type
        OpenGLViewer::CameraViewType camera_view =
            OpenGLViewer::CameraViewType::ISOMETRIC;
        //! Enable performance profiling
        bool enable_profiling = false;
    };

    // ----------------------------------------------------------------------------
    //! \brief Constructor.
    // ----------------------------------------------------------------------------
    explicit RobotViewerApplication(Configuration const& p_config);

    // ----------------------------------------------------------------------------
    //! \brief Get the error message.
    // ----------------------------------------------------------------------------
    std::string const& error() const
    {
        return m_error;
    }

private:

    // ----------------------------------------------------------------------------
    //! \brief Initialize the application. Called once at startup.
    //! \return true if initialization was successful, false otherwise.
    // ----------------------------------------------------------------------------
    bool onSetup() override;

    // ----------------------------------------------------------------------------
    //! \brief Render the application. Called every frame.
    // ----------------------------------------------------------------------------
    void onDraw() override;

    // ----------------------------------------------------------------------------
    //! \brief Update the application logic. Called every frame.
    //! \param dt Delta time in seconds since last frame.
    // ----------------------------------------------------------------------------
    void onUpdate(float const dt) override;

    // ----------------------------------------------------------------------------
    //! \brief Update physics simulation. Called at fixed time intervals.
    //! \param dt Fixed delta time for physics simulation.
    // ----------------------------------------------------------------------------
    void onPhysicUpdate(float const dt) override;

    // ----------------------------------------------------------------------------
    //! \brief Handle input events. Called for each input event.
    //! \param key The key that was pressed.
    //! \param action The action (press/release).
    // ----------------------------------------------------------------------------
    void handleInput(int key, int action) override;

private:

    // ----------------------------------------------------------------------------
    //! \brief Load robot from URDF file.
    //! \param p_urdf_file Path to the URDF file.
    //! \return true if loading was successful, false otherwise.
    // ----------------------------------------------------------------------------
    bool loadRobot(const std::string& p_urdf_file);

    // ----------------------------------------------------------------------------
    //! \brief Set the target joint for inverse kinematics. Usually is the tool
    //! center point (TCP).
    //! \param p_target_joint_name Name of the target joint.
    //! \return true if setting the target was successful, false otherwise.
    // ----------------------------------------------------------------------------
    bool setTarget(std::string const& p_target_joint_name);

    // ----------------------------------------------------------------------------
    //! \brief Set the initial camera view.
    //! \param p_look_at_joint_name Name of the joint to look at.
    //! \return true if setting the camera target was successful, false
    //! otherwise.
    // ----------------------------------------------------------------------------
    bool initCameraView(std::string const& p_look_at_joint_name);

    // ----------------------------------------------------------------------------
    //! \brief Animate the robot with sinusoidal motion.
    //! \param time Current time in seconds.
    // ----------------------------------------------------------------------------
    void handleAnimation(double p_time);

    // ----------------------------------------------------------------------------
    //! \brief Handle inverse kinematics control.
    // ----------------------------------------------------------------------------
    void handleInverseKinematics();

private:

    //! \brief Path searcher
    Path m_path;
    //! \brief Configuration
    Configuration m_config;
    //! \brief Robot to control and display
    std::unique_ptr<robotik::Robot> m_robot;
    //! \brief Control mode of the robot (animation or inverse kinematics)
    std::atomic<ControlMode> m_control_mode{ ControlMode::ANIMATION };
    //! \brief In inverse kinematics mode, set the joint used as the target for
    //! its control. Usually is the tool center point (TCP).
    const robotik::scene::Node* m_target = nullptr;
    //! \brief Target pose of the target joint (inverse kinematics)
    robotik::Pose m_target_pose = robotik::Pose::Zero();
    //! \brief Flag to indicate if the target has been updated.
    std::atomic<bool> m_target_updated{ false };
    //! \brief Set the joint to be tracked by the camera.
    const robotik::scene::Node* m_camera_target = nullptr;
    //! \brief Mutex to protect the robot
    std::mutex m_robot_mutex;
    //! \brief Start time of the application
    std::chrono::steady_clock::time_point m_start_time;
    //! \brief Error message
    std::string m_error;

    // Performance optimization: animation cache
    mutable std::vector<double> m_cached_joint_values;
    mutable double m_last_animation_time = -1.0;
    mutable bool m_animation_dirty = true;
};

} // namespace robotik