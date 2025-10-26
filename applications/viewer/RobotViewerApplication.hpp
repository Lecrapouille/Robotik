/**
 * @file RobotViewerApplication.hpp
 * @brief Robot viewer application class for the 3D viewer.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#pragma once

#include "Configuration.hpp"
#include "Robotik/Viewer/Application/OpenGLApplication.hpp"

#include "Robotik/Core/PhysicsSimulator.hpp"

#include "Robotik/Viewer/Camera.hpp"
#include "Robotik/Viewer/GeometryRenderer.hpp"
#include "Robotik/Viewer/MeshManager.hpp"
#include "Robotik/Viewer/RobotManager.hpp"
#include "Robotik/Viewer/ShaderManager.hpp"

namespace robotik::viewer::application
{

class OpenGLWindow;

// ****************************************************************************
//! \brief Robot viewer application that inherits from Application.
// ****************************************************************************
class RobotViewerApplication final: public OpenGLApplication
{
public:

    // ----------------------------------------------------------------------------
    //! \brief Constructor.
    // ----------------------------------------------------------------------------
    explicit RobotViewerApplication(Configuration const& p_config);

    // ----------------------------------------------------------------------------
    //! \brief Destructor.
    // ----------------------------------------------------------------------------
    // ~RobotViewerApplication() override;

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

private:

    // ----------------------------------------------------------------------------
    //! \brief Compute IK target poses.
    //! \param p_controlled_robot The controlled robot.
    // ----------------------------------------------------------------------------
    void
    computeIKTargetPoses(RobotManager::ControlledRobot& p_controlled_robot);

    // ----------------------------------------------------------------------------
    //! \brief Compute trajectory configurations.
    //! \param p_controlled_robot The controlled robot.
    // ----------------------------------------------------------------------------
    void
    computeTrajectoryConfigs(RobotManager::ControlledRobot& p_controlled_robot);

    // ----------------------------------------------------------------------------
    //! \brief Search for the joint to control or inverse kinematics, given by
    //! the user from the application command line. If not provided, find the
    //! robot end effector.
    //! \param p_controlled_robot The controlled robot.
    //! \param p_control_joint_name The name of the control joint.
    //! \return true if setting the control joint was successful, false
    //! otherwise.
    // ----------------------------------------------------------------------------
    bool setControlJoint(RobotManager::ControlledRobot& p_controlled_robot,
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
    bool setCameraTarget(RobotManager::ControlledRobot& p_controlled_robot,
                         std::string const& p_look_at_joint_name,
                         bool p_use_root_if_not_found) const;

    // ----------------------------------------------------------------------------
    //! \brief Setup shaders.
    //! \param p_program_name The name of the shader program.
    //! \return true if setting up the shader program was successful, false
    //! otherwise.
    // ----------------------------------------------------------------------------
    bool setupShaderProgram(std::string const& p_program_name);

    // ----------------------------------------------------------------------------
    //! \brief Render a geometry.
    //! \param p_geometry The geometry.
    //! \param p_transform The transform.
    // ----------------------------------------------------------------------------
    void renderGeometry(Geometry const& p_geometry,
                        Eigen::Matrix4f const& p_transform);

    // ----------------------------------------------------------------------------
    //! \brief Render trajectory path with axes.
    //! \param p_robot The controlled robot.
    // ----------------------------------------------------------------------------
    void renderTrajectoryPath(RobotManager::ControlledRobot& p_robot);

    // ----------------------------------------------------------------------------
    //! \brief Handle animation of the given robot.
    //! \param p_robot The robot.
    //! \param p_time The time.
    // ----------------------------------------------------------------------------
    void handleAnimation(Robot& p_robot, double p_time) const;

    // ----------------------------------------------------------------------------
    //! \brief Handle inverse kinematics.
    //! \param p_robot The controlled robot.
    // ----------------------------------------------------------------------------
    void handleInverseKinematics(RobotManager::ControlledRobot& p_robot);

    // ----------------------------------------------------------------------------
    //! \brief Handle trajectory following.
    //! \param p_robot The controlled robot.
    //! \param p_time Current time in seconds.
    //! \param p_dt Delta time in seconds.
    // ----------------------------------------------------------------------------
    void handleTrajectory(RobotManager::ControlledRobot& p_robot,
                          double p_time,
                          double p_dt);

    // ----------------------------------------------------------------------------
    //! \brief Update the camera target position for track the chosen robot
    //! element and update OpenGL shader with the camera matrices.
    // ----------------------------------------------------------------------------
    void updateCameraTarget();

    // ----------------------------------------------------------------------------
    //! \brief Setup robot callbacks for ImGui.
    // ----------------------------------------------------------------------------
    void setupImGuiCallbacks();

public:

    //! \brief Search paths to load URDF files.
    Path path;

private:

    // Configuration
    Configuration const& m_config;

    // Robot management callbacks
    std::function<bool(const std::string&)> m_load_robot_callback;
    std::function<bool(const std::string&)> m_remove_robot_callback;
    std::function<std::vector<std::string>()> m_robot_list_callback;
    std::function<std::vector<std::pair<std::string, double>>(
        const std::string&)>
        m_get_joints_callback;
    std::function<void(const std::string&, const std::string&, double)>
        m_set_joint_callback;
    std::function<std::vector<std::string>(const std::string&)>
        m_get_nodes_callback;
    std::function<void(const std::string&, int)> m_set_control_mode_callback;
    std::function<int(const std::string&)> m_get_control_mode_callback;
    std::function<void(const std::string&, const std::string&)>
        m_set_end_effector_callback;
    std::function<std::string(const std::string&)> m_get_end_effector_callback;
    std::function<void(const std::string&, const std::string&)>
        m_set_camera_target_callback;
    std::function<std::string(const std::string&)> m_get_camera_target_callback;

    // Components
    Camera m_camera;
    ShaderManager m_shader_manager;
    MeshManager m_mesh_manager;
    GeometryRenderer m_geometry_renderer;
    RobotManager m_robot_manager;
    PhysicsSimulator m_physics_simulator;

    // Cached OpenGL shader uniforms
    int m_model_uniform = -1;
    int m_color_uniform = -1;
    int m_projection_uniform = -1;
    int m_view_uniform = -1;

    // Animation
    std::chrono::steady_clock::time_point m_start_time;
    size_t m_target_pose_index = 0;

    // Title and FPS states
    std::string m_title;
    size_t m_fps = 0;
};

} // namespace robotik::viewer::application