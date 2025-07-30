#pragma once

#include "OpenGLViewer.hpp"
#include "Robotik/Robot.hpp"
#include "Robotik/private/Conversions.hpp"
#include <GLFW/glfw3.h>

#include <atomic>
#include <chrono>
#include <mutex>
#include <optional>
#include <string>

namespace robotik_viewer
{

// ----------------------------------------------------------------------------
//! \brief Control modes for the robot.
// ----------------------------------------------------------------------------
enum class ControlMode
{
    ANIMATION,         //!< Automatic sinusoidal animation
    INVERSE_KINEMATICS //!< Interactive inverse kinematics control
};

// ----------------------------------------------------------------------------
//! \brief Display information for on-screen overlay.
// ----------------------------------------------------------------------------
struct DisplayInfo
{
    ControlMode control_mode = ControlMode::ANIMATION;
    robotik::OpenGLViewer::CameraViewType camera_view =
        robotik::OpenGLViewer::CameraViewType::SIDE;
    robotik::Pose current_target_pose = robotik::Pose::Zero();
    bool show_help = true;
    std::string status_message;
};

// ----------------------------------------------------------------------------
//! \brief Global state for physics and control (thread-safe).
// ----------------------------------------------------------------------------
struct RobotState
{
    std::atomic<ControlMode> control_mode{ ControlMode::ANIMATION };
    robotik::Pose target_pose = robotik::Pose::Zero();
    std::atomic<bool> target_updated{ false };
    std::atomic<bool> physics_running{ true };
    std::mutex robot_mutex;   //!< Protects robot joint values
    std::mutex display_mutex; //!< Protects display information
    std::chrono::steady_clock::time_point start_time;
    DisplayInfo display_info;
};

// ----------------------------------------------------------------------------
//! \brief Robot animation and control functions.
// ----------------------------------------------------------------------------

//! \brief Animate the robot with sinusoidal motion (thread-safe version).
//! \param p_robot The robot to animate.
//! \param p_time Current time in seconds.
void animate(robotik::Robot& p_robot, double p_time);

//! \brief Handle inverse kinematics control.
//! \param p_robot The robot to control.
//! \param p_end_effector Pointer to the end effector node.
//! \param p_state Global robot state.
void handle_inverse_kinematics(robotik::Robot& p_robot,
                               const robotik::scene::Node* p_end_effector,
                               RobotState& p_state);

//! \brief Physics loop running at 1kHz.
//! \param p_robot Reference to the robot.
//! \param p_end_effector Pointer to the end effector node.
//! \param p_state Global robot state.
void physics_loop(robotik::Robot& p_robot,
                  const robotik::scene::Node* p_end_effector,
                  RobotState& p_state);

// ----------------------------------------------------------------------------
//! \brief Input handling functions.
// ----------------------------------------------------------------------------

//! \brief Handle camera control inputs (1-5 keys).
//! \param p_key The key that was pressed.
//! \param p_action The action (press/release).
//! \param p_state Global robot state.
//! \return The selected camera view type, or std::nullopt if no change.
std::optional<robotik::OpenGLViewer::CameraViewType>
handle_camera_input(int p_key, int p_action, RobotState& p_state);

//! \brief Handle mode switching input (M key).
//! \param p_key The key that was pressed.
//! \param p_action The action (press/release).
//! \param p_end_effector Pointer to end effector for IK initialization.
//! \param p_state Global robot state.
//! \return True if mode was changed.
bool handle_mode_switch(int p_key,
                        int p_action,
                        const robotik::scene::Node* p_end_effector,
                        RobotState& p_state);

//! \brief Handle inverse kinematics target control (AZERTY: ZQSD + AE keys).
//! \param p_key The key that was pressed.
//! \param p_action The action (press/release).
//! \param p_state Global robot state.
//! \return True if target was updated.
bool handle_ik_target_input(int p_key, int p_action, RobotState& p_state);

//! \brief Handle help display toggle (H key).
//! \param p_key The key that was pressed.
//! \param p_action The action (press/release).
//! \param p_state Global robot state.
//! \return True if help display was toggled.
bool handle_help_toggle(int p_key, int p_action, RobotState& p_state);

//! \brief Main input handler that delegates to specific handlers.
//! \param p_key The key that was pressed.
//! \param p_end_effector Pointer to end effector for IK initialization.
//! \param p_state Global robot state.
//! \param p_viewer Reference to the OpenGL viewer for key state access.
//! \return The selected camera view type, or std::nullopt if no camera change.
std::optional<robotik::OpenGLViewer::CameraViewType>
handle_input(int p_key,
             const robotik::scene::Node* p_end_effector,
             RobotState& p_state,
             const robotik::OpenGLViewer& p_viewer);

//! \brief Handle continuous input for held keys (called in main loop).
//! \param p_state Global robot state.
//! \param p_viewer Reference to the OpenGL viewer for key state access.
void handle_continuous_input(RobotState& p_state,
                             const robotik::OpenGLViewer& p_viewer);

// ----------------------------------------------------------------------------
//! \brief Display and UI functions.
// ----------------------------------------------------------------------------

//! \brief Get string representation of control mode.
//! \param p_mode The control mode.
//! \return String representation.
std::string control_mode_to_string(ControlMode p_mode);

//! \brief Get string representation of camera view.
//! \param p_view The camera view type.
//! \return String representation.
std::string camera_view_to_string(robotik::OpenGLViewer::CameraViewType p_view);

//! \brief Print current status to console.
//! \param p_state Global robot state.
void print_status(const RobotState& p_state);

//! \brief Print help information.
void print_help();

//! \brief Display usage information.
//! \param p_program_name The program name.
void display_usage(const std::string& p_program_name);

} // namespace robotik_viewer