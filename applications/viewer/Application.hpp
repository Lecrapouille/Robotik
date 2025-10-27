/**
 * @file Application.hpp
 * @brief Robot viewer application class for the 3D viewer.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#pragma once

#include "Configuration.hpp"
#include "DearRobotHMI.hpp"

#include "Robotik/Simulation/PhysicsSimulator.hpp"

#include "Robotik/Renderer/Application/OpenGLApplication.hpp"
#include "Robotik/Renderer/Camera/OrbitController.hpp"
#include "Robotik/Renderer/Camera/PerspectiveCamera.hpp"
#include "Robotik/Renderer/Managers/MeshManager.hpp"
#include "Robotik/Renderer/Managers/RobotManager.hpp"
#include "Robotik/Renderer/Managers/ShaderManager.hpp"
#include "Robotik/Renderer/RenderVisitor.hpp"
#include "Robotik/Renderer/Renderer.hpp"

#include "Robotik/Common/Path.hpp"

namespace robotik::application
{

// ****************************************************************************
//! \brief Robot viewer application that inherits from Application.
// ****************************************************************************
class Application final: public renderer::OpenGLApplication
{
public:

    // ----------------------------------------------------------------------------
    //! \brief Constructor.
    // ----------------------------------------------------------------------------
    explicit Application(Configuration const& p_config);

    // ----------------------------------------------------------------------------
    //! \brief Run the application with the given configuration.
    //! \return true if the application was run successfully, false otherwise.
    // ----------------------------------------------------------------------------
    bool run();

private: // override OpenGLApplication methods

    // ----------------------------------------------------------------------------
    //! \brief Set the title of the application.
    //! \param p_title The title of the application.
    // ----------------------------------------------------------------------------
    void setTitle(std::string const& p_title) override;

    // ----------------------------------------------------------------------------
    //! \brief Initialize the application. Called once at startup. This method
    //! is called before the physics thread is started.
    //! \return true if initialization was successful, false otherwise. In this
    //! last case, you can set the error message using the error() method.
    // ----------------------------------------------------------------------------
    bool onSetup() override;

    // ----------------------------------------------------------------------------
    //! \brief Cleanup the application. Called once at shutdown. This method
    //! is called after the physics thread is stopped.
    //! You can use this method to clean up any resources you allocated in the
    //! onSetup() method.
    // ----------------------------------------------------------------------------
    void onTeardown() override;

    // ----------------------------------------------------------------------------
    //! \brief Render the 3D scene. Called automatically within the viewport.
    // ----------------------------------------------------------------------------
    void onDrawScene() override;

    // ----------------------------------------------------------------------------
    //! \brief Draw ImGui menu bar.
    // ----------------------------------------------------------------------------
    void onDrawMenuBar() override;

    // ----------------------------------------------------------------------------
    //! \brief Draw ImGui main control panel.
    // ----------------------------------------------------------------------------
    void onDrawMainPanel() override;

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
    //! \brief Update the FPS.
    //! \param p_fps Current FPS value.
    // ----------------------------------------------------------------------------
    void onFPSUpdated(size_t const p_fps) override;

    // ----------------------------------------------------------------------------
    //! \brief Handle key event (override from Application).
    //! \param p_key Key code.
    //! \param p_scancode Scan code.
    //! \param p_action Action (press, release, repeat).
    //! \param p_mods Modifier keys.
    // ----------------------------------------------------------------------------
    void
    onKeyInput(int p_key, int p_scancode, int p_action, int p_mods) override;

    // ----------------------------------------------------------------------------
    //! \brief Handle scroll event.
    //! \param p_xoffset Scroll x offset.
    //! \param p_yoffset Scroll y offset.
    // ----------------------------------------------------------------------------
    void onScroll(double p_xoffset, double p_yoffset) override;

    // ----------------------------------------------------------------------------
    //! \brief Handle window resize event.
    //! \param p_width New window width.
    //! \param p_height New window height.
    // ----------------------------------------------------------------------------
    void onWindowResize(int p_width, int p_height) override;

    // ----------------------------------------------------------------------------
    //! \brief Handle mouse button event.
    //! \param p_button Mouse button.
    //! \param p_action Action (press, release).
    //! \param p_mods Modifier keys.
    // ----------------------------------------------------------------------------
    void onMouseButton(int p_button, int p_action, int p_mods) override;

    // ----------------------------------------------------------------------------
    //! \brief Handle cursor position event.
    //! \param p_xpos Cursor x position.
    //! \param p_ypos Cursor y position.
    // ----------------------------------------------------------------------------
    void onCursorPos(double p_xpos, double p_ypos) override;

private:

    // ----------------------------------------------------------------------------
    //! \brief Compute IK target poses.
    //! \param p_controlled_robot The controlled robot.
    // ----------------------------------------------------------------------------
    void computeIKTargetPoses(
        renderer::RobotManager::ControlledRobot& p_controlled_robot);

    // ----------------------------------------------------------------------------
    //! \brief Compute trajectory configurations.
    //! \param p_controlled_robot The controlled robot.
    // ----------------------------------------------------------------------------
    void computeTrajectoryConfigs(
        renderer::RobotManager::ControlledRobot& p_controlled_robot);

    // ----------------------------------------------------------------------------
    //! \brief Search for the joint to control for inverse kinematics, given by
    //! the user from the application command line. If not provided, find the
    //! robot end effector.
    //! \param p_controlled_robot The controlled robot.
    //! \param p_control_joint_name The name of the control joint.
    //! \return true if setting the control joint was successful, false
    //! otherwise.
    // ----------------------------------------------------------------------------
    bool
    setControlJoint(renderer::RobotManager::ControlledRobot& p_controlled_robot,
                    std::string const& p_control_joint_name) const;

    // ----------------------------------------------------------------------------
    //! \brief Search for the joint to look at, given by the user from the
    //! application command line. If not provided, use the robot root.
    //! \param p_controlled_robot The controlled robot.
    //! \param p_look_at_joint_name The name of the look at joint.
    //! \param p_use_root_if_not_found Whether to use the root if not found.
    //! \return true if setting the look at joint was successful, false
    //! otherwise.
    // ----------------------------------------------------------------------------
    bool
    setCameraTarget(renderer::RobotManager::ControlledRobot& p_controlled_robot,
                    std::string const& p_look_at_joint_name,
                    bool p_use_root_if_not_found) const;

    // ----------------------------------------------------------------------------
    //! \brief Render trajectory path with axes using RenderVisitor.
    //! \param p_robot The controlled robot.
    // ----------------------------------------------------------------------------
    void renderTrajectoryPath(renderer::RobotManager::ControlledRobot& p_robot);

    // ----------------------------------------------------------------------------
    //! \brief Handle animation of the given robot.
    //! \param p_robot The controlled robot.
    //! \param p_time The time.
    // ----------------------------------------------------------------------------
    void handleAnimation(renderer::RobotManager::ControlledRobot& p_robot,
                         double p_time) const;

    // ----------------------------------------------------------------------------
    //! \brief Handle inverse kinematics.
    //! \param p_robot The controlled robot.
    // ----------------------------------------------------------------------------
    void
    handleInverseKinematics(renderer::RobotManager::ControlledRobot& p_robot);

    // ----------------------------------------------------------------------------
    //! \brief Handle trajectory following.
    //! \param p_robot The controlled robot.
    //! \param p_time Current time in seconds.
    //! \param p_dt Delta time in seconds.
    // ----------------------------------------------------------------------------
    void handleTrajectory(renderer::RobotManager::ControlledRobot& p_robot,
                          double p_time,
                          double p_dt);

private:

    //! \brief DearImGui-based HMI for robot control and visualization.
    std::unique_ptr<DearRobotHMI> m_hmi;
    //! \brief Application settings and configuration.
    Configuration const& m_config;
    //! \brief Search paths to load URDF, STL, etc. files.
    Path m_path;
    //! \brief Perspective camera for intuitive robot inspection.
    std::unique_ptr<renderer::PerspectiveCamera> m_perspective_camera;
    //! \brief Orbit controller for orbiting around the robot.
    std::unique_ptr<renderer::OrbitController> m_orbit_controller;
    //! \brief Renderer for rendering the robot.
    std::unique_ptr<renderer::Renderer> m_renderer;
    //! \brief Shader manager for managing shaders.
    renderer::ShaderManager m_shader_manager;
    //! \brief Mesh manager for managing meshes.
    renderer::MeshManager m_mesh_manager;
    //! \brief Robot manager for managing robots.
    renderer::RobotManager m_robot_manager;
    //! \brief Physics simulator for simulating physics.
    robotik::PhysicsSimulator m_physics_simulator;
    //! \brief Cached OpenGL shader uniforms.
    int m_projection_uniform = -1;
    //! \brief Cached OpenGL shader uniforms.
    int m_view_uniform = -1;
    //! \brief Start time for animation.
    std::chrono::steady_clock::time_point m_start_time;
    //! \brief Target pose index for animation.
    size_t m_target_pose_index = 0;
    //! \brief Title of the application.
    std::string m_title;
    //! \brief FPS of the application.
    size_t m_fps = 0;
};

} // namespace robotik::application