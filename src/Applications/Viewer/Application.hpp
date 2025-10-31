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
#include "Controller.hpp"
#include "HMI.hpp"

#include "Robotik/Core/Simulation/PhysicsSimulator.hpp"

#include "Robotik/Renderer/Application/OpenGLApplication.hpp"
#include "Robotik/Renderer/Camera/OrbitController.hpp"
#include "Robotik/Renderer/Camera/PerspectiveCamera.hpp"
#include "Robotik/Renderer/Managers/MeshManager.hpp"
#include "Robotik/Renderer/Managers/RobotManager.hpp"
#include "Robotik/Renderer/Managers/ShaderManager.hpp"
#include "Robotik/Renderer/RenderVisitor.hpp"

#include "Robotik/Core/Common/Path.hpp"

#include <array>

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

private: // setups

    // ----------------------------------------------------------------------------
    //! \brief Setup the camera.
    // ----------------------------------------------------------------------------
    void setupCamera();

    // ----------------------------------------------------------------------------
    //! \brief Setup the main shader.
    // ----------------------------------------------------------------------------
    bool setupMainShader();

    // ----------------------------------------------------------------------------
    //! \brief Setup the renderer.
    // ----------------------------------------------------------------------------
    bool setupRenderer();

    // ----------------------------------------------------------------------------
    //! \brief Setup a robot.
    // ----------------------------------------------------------------------------
    bool setupRobot(std::string const& p_urdf_file);

private:

    // ----------------------------------------------------------------------------
    //! \brief Render a robot.
    //! \param p_robot The controlled robot.
    //! \param p_is_selected Whether this robot is the currently selected one.
    // ----------------------------------------------------------------------------
    void renderRobot(renderer::RobotManager::ControlledRobot const& p_robot,
                     bool p_is_selected = false);

    // ----------------------------------------------------------------------------
    //! \brief Render trajectory path with axes.
    //! \param p_robot The controlled robot.
    // ----------------------------------------------------------------------------
    void renderTrajectoryPath(
        renderer::RobotManager::ControlledRobot const& p_robot);

    // ----------------------------------------------------------------------------
    //! \brief Set camera target for a robot.
    //! \param p_robot The robot.
    //! \param p_node_name Node name.
    //! \param p_use_root_if_not_found Whether to use root if not found.
    //! \return true if successful.
    // ----------------------------------------------------------------------------
    bool setCameraTarget(renderer::RobotManager::ControlledRobot& p_robot,
                         std::string const& p_node_name,
                         bool p_use_root_if_not_found) const;

    // ----------------------------------------------------------------------------
    //! \brief Switch neutral position for the current robot.
    // ----------------------------------------------------------------------------
    void switchNeutralPosition() const;

    // ----------------------------------------------------------------------------
    //! \brief Switch visibility for the current robot.
    // ----------------------------------------------------------------------------
    void switchVisibility() const;

private:

    //! \brief DearImGui-based HMI for robot control and visualization.
    std::unique_ptr<HMI> m_hmi;
    //! \brief Model-View-Controller controller for managing robot logic.
    std::unique_ptr<Controller> m_controller;
    //! \brief Application settings and configuration.
    Configuration const& m_config;
    //! \brief Search paths to load URDF, STL, etc. files.
    Path m_path;
    //! \brief Perspective camera for intuitive robot inspection.
    std::unique_ptr<renderer::PerspectiveCamera> m_perspective_camera;
    //! \brief Orbit controller for orbiting around the robot.
    std::unique_ptr<renderer::OrbitController> m_orbit_controller;
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
    //! \brief Viewport for the application.
    std::array<int, 4> m_viewport{ 0, 0, 0, 0 };
    //! \brief Start time for animation.
    std::chrono::steady_clock::time_point m_start_time;
    //! \brief Title of the application.
    std::string m_title;
    //! \brief FPS of the application.
    size_t m_fps = 0;
};

} // namespace robotik::application