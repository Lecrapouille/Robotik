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

#include "Robotik/Viewer/Application.hpp"
#include "Robotik/Viewer/Camera.hpp"
#include "Robotik/Viewer/GeometryRenderer.hpp"
#include "Robotik/Viewer/MeshManager.hpp"
#include "Robotik/Viewer/OpenGLWindow.hpp"
#include "Robotik/Viewer/RobotManager.hpp"
#include "Robotik/Viewer/ShaderManager.hpp"

namespace robotik::viewer
{

class OpenGLWindow;

// ****************************************************************************
//! \brief Robot viewer application that inherits from Application.
// ****************************************************************************
class RobotViewerApplication: public Application
{
public:

    // ----------------------------------------------------------------------------
    //! \brief Constructor.
    // ----------------------------------------------------------------------------
    explicit RobotViewerApplication(Configuration const& p_config);

    // ----------------------------------------------------------------------------
    //! \brief Destructor.
    // ----------------------------------------------------------------------------
    ~RobotViewerApplication() override;

    // ----------------------------------------------------------------------------
    //! \brief Run the application with the given configuration.
    //! \return true if the application was run successfully, false otherwise.
    // ----------------------------------------------------------------------------
    bool run();

private: // override Application methods

    // ----------------------------------------------------------------------------
    //! \brief Set the title of the application.
    //! \param p_title The title of the application.
    // ----------------------------------------------------------------------------
    void setTitle(std::string const& p_title) override;

    // ----------------------------------------------------------------------------
    //! \brief Check if the application should close.
    //! \return true if the application should close, false otherwise.
    // ----------------------------------------------------------------------------
    bool isHalting() const override;

    // ----------------------------------------------------------------------------
    //! \brief Initialize the application. Called once at startup.
    //! \return true if initialization was successful, false otherwise.
    // ----------------------------------------------------------------------------
    bool onSetup() override;

    // ----------------------------------------------------------------------------
    //! \brief Cleanup the application. Called once at shutdown.
    // ----------------------------------------------------------------------------
    void onCleanup() override;

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
    //! \brief Update the FPS.
    //! \param p_fps Current FPS value.
    // ----------------------------------------------------------------------------
    void onFPSUpdated(size_t const p_fps) override;

    // ----------------------------------------------------------------------------
    //! \brief Handle window resize event.
    //! \param width New window width.
    //! \param height New window height.
    // ----------------------------------------------------------------------------
    void onWindowResize(int width, int height);

    // ----------------------------------------------------------------------------
    //! \brief Handle key event.
    //! \param key Key code.
    //! \param scancode Scan code.
    //! \param action Action (press, release, repeat).
    //! \param mods Modifier keys.
    // ----------------------------------------------------------------------------
    void onKeyInput(int key, int scancode, int action, int mods);

    // ----------------------------------------------------------------------------
    //! \brief Handle mouse button event.
    //! \param button Mouse button.
    //! \param action Action (press, release).
    //! \param mods Modifier keys.
    // ----------------------------------------------------------------------------
    void onMouseButton(int button, int action, int mods);

    // ----------------------------------------------------------------------------
    //! \brief Handle cursor position event.
    //! \param xpos Cursor x position.
    //! \param ypos Cursor y position.
    // ----------------------------------------------------------------------------
    void onCursorPos(double xpos, double ypos);

    // ----------------------------------------------------------------------------
    //! \brief Handle scroll event.
    //! \param xoffset Scroll x offset.
    //! \param yoffset Scroll y offset.
    // ----------------------------------------------------------------------------
    void onScroll(double xoffset, double yoffset);

private:

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
                         std::string const& p_control_joint_name);

    // ----------------------------------------------------------------------------
    //! \brief Search for the joint to look at, given by the user from the
    //! application command line. If not provided, use the robot root.
    //! \param p_controlled_robot The controlled robot.
    //! \param p_look_at_joint_name The name of the look at joint.
    //! \return true if setting the look at joint was successful, false
    //! otherwise.
    // ----------------------------------------------------------------------------
    bool initCameraView(RobotManager::ControlledRobot& p_controlled_robot,
                        std::string const& p_look_at_joint_name);

    // ----------------------------------------------------------------------------
    //! \brief Setup shaders.
    //! \param p_program_name The name of the shader program.
    //! \return true if setting up the shaders was successful, false otherwise.
    // ----------------------------------------------------------------------------
    bool setupShaders(std::string const& p_program_name);

    // ----------------------------------------------------------------------------
    //! \brief Render a geometry.
    // ----------------------------------------------------------------------------
    void renderGeometry(Geometry const& p_geometry,
                        Eigen::Matrix4f const& p_transform);

    // ----------------------------------------------------------------------------
    //! \brief Handle animation of the given robot.
    //! \param p_robot The robot.
    //! \param p_time The time.
    // ----------------------------------------------------------------------------
    void handleAnimation(Robot& p_robot, double p_time);

    // ----------------------------------------------------------------------------
    //! \brief Handle inverse kinematics.
    // ----------------------------------------------------------------------------
    void handleInverseKinematics();

private:

    // Configuration
    Configuration const& m_config;

    // Components
    OpenGLWindow m_window;
    Camera m_camera;
    ShaderManager m_shader_manager;
    MeshManager m_mesh_manager;
    GeometryRenderer m_geometry_renderer;
    RobotManager m_robot_manager;

    // Cached OpenGL shader uniforms
    int m_model_uniform = -1;
    int m_color_uniform = -1;
    int m_projection_uniform = -1;
    int m_view_uniform = -1;

    // Animation
    std::chrono::steady_clock::time_point m_start_time;

    // Title and FPS states
    std::string m_title;
    size_t m_fps = 0;
};

} // namespace robotik::viewer