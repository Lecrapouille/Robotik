/**
 * @file Application.hpp
 * @brief Robot simulator application class for the 3D simulator.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#pragma once

#include "Configuration.hpp"
#include "ImGuiView.hpp"
#include "RobotController.hpp"

#include "Robotik/Core/Simulation/PhysicsSimulator.hpp"
#include "Robotik/Renderer/Application/OpenGLApplication.hpp"
#include "Robotik/Renderer/Managers/GeometryManager.hpp"
#include "Robotik/Renderer/Managers/ShaderManager.hpp"
#include "Robotik/Renderer/RenderVisitor.hpp"

#include "Robotik/Core/Common/Path.hpp"

#include <memory>

namespace robotik::application
{

// Forward declaration
class ImGuiView;

// ****************************************************************************
//! \brief Concrete application that inherits from OpenGL Application.
//!
//! This class is the main entry point for the simulator application.
//! It provides a complete 3D simulator with robot visualization, physics
//! simulation, and human-machine interface (teach pendant emulation: joint
//! control, Cartesian control, waypoints saving and playback).
// ****************************************************************************
class MainApplication final: public renderer::OpenGLApplication
{
public:

    // ----------------------------------------------------------------------------
    //! \brief Constructor. Only calls the constructor of the OpenGL
    //! Application.
    //! \param p_config The configuration of the application (window width,
    //! height, title, search paths, target FPS, target physics Hz ...).
    // ----------------------------------------------------------------------------
    explicit MainApplication(Configuration const& p_config);

    // ----------------------------------------------------------------------------
    //! \brief Run the application with the given configuration.
    //! This method is blocking and will return when the application halted
    //! (e.g. by user request or error).
    //! \return true if the application was run successfully, false otherwise.
    //! \note This method calls the run() method of the OpenGL Application but
    //! using the target FPS and physics Hz from the configuration.
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
    //! \brief Setup the main shader.
    // ----------------------------------------------------------------------------
    bool setupMainShader();

    // ----------------------------------------------------------------------------
    //! \brief Setup the builting meshes.
    // ----------------------------------------------------------------------------
    bool setupMeshes();

    // ----------------------------------------------------------------------------
    //! \brief Setup the render visitor.
    // ----------------------------------------------------------------------------
    void setupRender();

    // ----------------------------------------------------------------------------
    //! \brief Setup the physics simulator.
    // ----------------------------------------------------------------------------
    void setupPhysicsSimulator();

    // ----------------------------------------------------------------------------
    //! \brief Setup the robots.
    // ----------------------------------------------------------------------------
    bool setupRobots();

    // ----------------------------------------------------------------------------
    //! \brief Setup the ImGui view.
    // ----------------------------------------------------------------------------
    bool setupImGuiView();

private:

    // ----------------------------------------------------------------------------
    //! \brief Render a robot and its teach pendant visualization if selected.
    //! \param p_robot The controlled robot to render.
    //! \param p_is_selected Whether this is the selected robot.
    // ----------------------------------------------------------------------------
    void renderRobot(ControlledRobot const& p_robot, bool p_is_selected) const;

    // ----------------------------------------------------------------------------
    //! \brief Render waypoints for the given robot.
    //! \param p_robot The controlled robot whose waypoints to render.
    // ----------------------------------------------------------------------------
    void renderWaypoints(ControlledRobot const& p_robot) const;

    // ----------------------------------------------------------------------------
    //! \brief Switch neutral position for the current robot.
    // ----------------------------------------------------------------------------
    void switchNeutralPosition() const;

    // ----------------------------------------------------------------------------
    //! \brief Switch visibility for the current robot.
    // ----------------------------------------------------------------------------
    void switchVisibility() const;

    // ----------------------------------------------------------------------------
    //! \brief Get the controlled robot by name.
    //! \param p_robot_name Robot name.
    //! \return Pointer to controlled robot, nullptr if not found.
    // ----------------------------------------------------------------------------
    ControlledRobot* getControlledRobot(std::string const& p_robot_name) const;

private:

    //! \brief Application settings and configuration.
    Configuration const& m_config;
    //! \brief Search paths to load URDF, STL, etc. files.
    Path m_path;
    //! \brief Robot controller for managing robots and their interactions.
    std::unique_ptr<RobotController> m_robot_controller;
    //! \brief ImGui view for managing robot control and visualization.
    std::unique_ptr<ImGuiView> m_imgui_view;
    //! \brief Shader manager for managing shaders.
    std::unique_ptr<renderer::ShaderManager> m_shader_manager;
    //! \brief Geometry manager for managing robot geometries and meshes.
    std::unique_ptr<renderer::GeometryManager> m_geometry_manager;
    //! \brief Mesh manager for managing meshes.
    std::unique_ptr<renderer::RenderVisitor> m_render;
    //! \brief Physics simulator for simulating physics.
    std::unique_ptr<PhysicsSimulator> m_physics_simulator;
    //! \brief Cached OpenGL shader uniforms.
    int m_projection_uniform = -1;
    //! \brief Cached OpenGL shader uniforms.
    int m_view_uniform = -1;
    //! \brief Title of the application.
    std::string m_title;
    //! \brief FPS of the application.
    size_t m_fps = 0;
};

} // namespace robotik::application