#pragma once

#include "Configuration.hpp"
#include "Robotik/Renderer/Application/OpenGLApplication.hpp"

#include <chrono>
#include <memory>

// Forward declarations
namespace robotik
{
class Robot;
}

namespace robotik::renderer
{
// Forward declarations
class PerspectiveCamera;
class OrbitController;
class ShaderManager;
class MeshManager;
class Renderer;
} // namespace robotik::renderer

namespace robotik::renderer::application
{

// ****************************************************************************
//! \brief Robot viewer application that inherits from OpenGLApplication.
// ****************************************************************************
class RobotViewerApplication: public OpenGLApplication
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

private: // override OpenGLApplication methods

    // ----------------------------------------------------------------------------
    //! \brief Initialize rendering systems.
    //! \return true if successful.
    // ----------------------------------------------------------------------------
    bool onSetup() override;

    // ----------------------------------------------------------------------------
    //! \brief Cleanup rendering systems.
    //! \return true if successful.
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

    // ----------------------------------------------------------------------------
    //! \brief Handle scroll event.
    //! \param p_xoffset Horizontal scroll offset.
    //! \param p_yoffset Vertical scroll offset.
    // ----------------------------------------------------------------------------
    void onScroll(double p_xoffset, double p_yoffset) override;

private:

    // Configuration
    Configuration const& m_config;

    // FPS state
    size_t m_fps = 0;

    // Rendering components
    std::unique_ptr<PerspectiveCamera> m_camera;
    std::unique_ptr<OrbitController> m_camera_controller;
    std::unique_ptr<ShaderManager> m_shader_manager;
    std::unique_ptr<MeshManager> m_mesh_manager;
    std::unique_ptr<Renderer> m_renderer;

    // Example: rotation angle for box
    float m_rotation_angle = 0.0f;

    // Robot
    std::unique_ptr<robotik::Robot> m_robot;
};

} // namespace robotik::renderer::application