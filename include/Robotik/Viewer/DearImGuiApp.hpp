/**
 * @file DearImGuiApp.hpp
 * @brief Dear ImGui application wrapper with docking support.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#pragma once

#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

#include <functional>
#include <string>
#include <utility>
#include <vector>

namespace robotik::viewer
{

// ****************************************************************************
//! \brief Dear ImGui application wrapper providing docking and viewport
//! integration.
//!
//! This class wraps Dear ImGui to provide:
//! - Docking support for flexible UI layout
//! - Multi-viewport support for detachable windows
//! - Framebuffer rendering for OpenGL content in ImGui windows
//! - Customizable rendering callback for 3D content
// ****************************************************************************
class ImGuiApp
{
public:

    using RenderCallback = std::function<void()>;
    using LoadRobotCallback = std::function<bool(const std::string&)>;
    using RemoveRobotCallback = std::function<bool(const std::string&)>;
    using RobotListCallback = std::function<std::vector<std::string>()>;
    using GetJointsCallback =
        std::function<std::vector<std::pair<std::string, double>>(
            const std::string&)>;
    using SetJointCallback =
        std::function<void(const std::string&, const std::string&, double)>;
    using GetNodesCallback =
        std::function<std::vector<std::string>(const std::string&)>;
    using SetControlModeCallback = std::function<void(const std::string&, int)>;
    using GetControlModeCallback = std::function<int(const std::string&)>;
    using SetEndEffectorCallback =
        std::function<void(const std::string&, const std::string&)>;
    using GetEndEffectorCallback =
        std::function<std::string(const std::string&)>;
    using SetCameraTargetCallback =
        std::function<void(const std::string&, const std::string&)>;
    using GetCameraTargetCallback =
        std::function<std::string(const std::string&)>;

    // ------------------------------------------------------------------------
    //! \brief Constructor.
    //! \param p_width Initial framebuffer width.
    //! \param p_height Initial framebuffer height.
    // ------------------------------------------------------------------------
    explicit ImGuiApp(size_t const p_width, size_t const p_height);

    // ------------------------------------------------------------------------
    //! \brief Destructor.
    // ------------------------------------------------------------------------
    ~ImGuiApp();

    // ------------------------------------------------------------------------
    //! \brief Set the render callback for 3D scene rendering.
    //! \param p_callback Function to call for rendering 3D content.
    // ------------------------------------------------------------------------
    void setRenderCallback(RenderCallback p_callback)
    {
        m_render_callback = p_callback;
    }

    // ------------------------------------------------------------------------
    //! \brief Set callback for loading a robot from URDF file.
    //! \param p_callback Function to call when loading a robot.
    // ------------------------------------------------------------------------
    void setLoadRobotCallback(LoadRobotCallback p_callback)
    {
        m_load_robot_callback = p_callback;
    }

    // ------------------------------------------------------------------------
    //! \brief Set callback for removing a robot.
    //! \param p_callback Function to call when removing a robot.
    // ------------------------------------------------------------------------
    void setRemoveRobotCallback(RemoveRobotCallback p_callback)
    {
        m_remove_robot_callback = p_callback;
    }

    // ------------------------------------------------------------------------
    //! \brief Set callback for getting the list of robots.
    //! \param p_callback Function to call to get robot list.
    // ------------------------------------------------------------------------
    void setRobotListCallback(RobotListCallback p_callback)
    {
        m_robot_list_callback = p_callback;
    }

    // ------------------------------------------------------------------------
    //! \brief Set callback for getting joint list and values.
    //! \param p_callback Function to call to get joints.
    // ------------------------------------------------------------------------
    void setGetJointsCallback(GetJointsCallback p_callback)
    {
        m_get_joints_callback = p_callback;
    }

    // ------------------------------------------------------------------------
    //! \brief Set callback for setting joint value.
    //! \param p_callback Function to call when joint value changes.
    // ------------------------------------------------------------------------
    void setSetJointCallback(SetJointCallback p_callback)
    {
        m_set_joint_callback = p_callback;
    }

    // ------------------------------------------------------------------------
    //! \brief Set callback for getting node list.
    //! \param p_callback Function to call to get nodes.
    // ------------------------------------------------------------------------
    void setGetNodesCallback(GetNodesCallback p_callback)
    {
        m_get_nodes_callback = p_callback;
    }

    // ------------------------------------------------------------------------
    //! \brief Set callback for setting control mode.
    //! \param p_callback Function to call when control mode changes.
    // ------------------------------------------------------------------------
    void setSetControlModeCallback(SetControlModeCallback p_callback)
    {
        m_set_control_mode_callback = p_callback;
    }

    // ------------------------------------------------------------------------
    //! \brief Set callback for getting control mode.
    //! \param p_callback Function to call to get control mode.
    // ------------------------------------------------------------------------
    void setGetControlModeCallback(GetControlModeCallback p_callback)
    {
        m_get_control_mode_callback = p_callback;
    }

    // ------------------------------------------------------------------------
    //! \brief Set callback for setting end effector.
    //! \param p_callback Function to call when end effector changes.
    // ------------------------------------------------------------------------
    void setSetEndEffectorCallback(SetEndEffectorCallback p_callback)
    {
        m_set_end_effector_callback = p_callback;
    }

    // ------------------------------------------------------------------------
    //! \brief Set callback for getting end effector.
    //! \param p_callback Function to call to get end effector.
    // ------------------------------------------------------------------------
    void setGetEndEffectorCallback(GetEndEffectorCallback p_callback)
    {
        m_get_end_effector_callback = p_callback;
    }

    // ------------------------------------------------------------------------
    //! \brief Set callback for setting camera target.
    //! \param p_callback Function to call when camera target changes.
    // ------------------------------------------------------------------------
    void setSetCameraTargetCallback(SetCameraTargetCallback p_callback)
    {
        m_set_camera_target_callback = p_callback;
    }

    // ------------------------------------------------------------------------
    //! \brief Set callback for getting camera target.
    //! \param p_callback Function to call to get camera target.
    // ------------------------------------------------------------------------
    void setGetCameraTargetCallback(GetCameraTargetCallback p_callback)
    {
        m_get_camera_target_callback = p_callback;
    }

    // ------------------------------------------------------------------------
    //! \brief Initialize ImGui context and backends.
    //! \return true if initialization successful, false otherwise.
    // ------------------------------------------------------------------------
    bool setup();

    // ------------------------------------------------------------------------
    //! \brief Cleanup ImGui context and resources.
    // ------------------------------------------------------------------------
    void teardown();

    // ------------------------------------------------------------------------
    //! \brief Draw the ImGui frame with dockspace and viewports.
    // ------------------------------------------------------------------------
    void draw();

    // ------------------------------------------------------------------------
    //! \brief Get ImGui IO structure for configuration.
    //! \return Reference to ImGui IO structure.
    // ------------------------------------------------------------------------
    ImGuiIO& io();

    // ------------------------------------------------------------------------
    //! \brief Enable or disable docking support.
    //! \param p_enable true to enable docking, false to disable.
    // ------------------------------------------------------------------------
    void enableDocking(bool p_enable = true);

    // ------------------------------------------------------------------------
    //! \brief Enable or disable multi-viewport support.
    //! \param p_enable true to enable viewports, false to disable.
    // ------------------------------------------------------------------------
    void enableViewports(bool p_enable = true);

    // ------------------------------------------------------------------------
    //! \brief Get the current viewport size.
    //! \return Viewport size in pixels.
    // ------------------------------------------------------------------------
    ImVec2 getViewportSize() const
    {
        return m_viewportSize;
    }

    // ------------------------------------------------------------------------
    //! \brief Get the current viewport width.
    //! \return Viewport width in pixels.
    // ------------------------------------------------------------------------
    int getViewportWidth() const
    {
        return static_cast<int>(m_viewportSize.x);
    }

    // ------------------------------------------------------------------------
    //! \brief Get the current viewport height.
    //! \return Viewport height in pixels.
    // ------------------------------------------------------------------------
    int getViewportHeight() const
    {
        return static_cast<int>(m_viewportSize.y);
    }

    // ------------------------------------------------------------------------
    //! \brief Get the current viewport position.
    //! \return Viewport position in screen coordinates.
    // ------------------------------------------------------------------------
    ImVec2 getViewportPos() const
    {
        return m_viewportPos;
    }

    // ------------------------------------------------------------------------
    //! \brief Check if the viewport is hovered by the mouse.
    //! \return true if viewport is hovered, false otherwise.
    // ------------------------------------------------------------------------
    bool isViewportHovered() const
    {
        return m_viewportHovered;
    }

    // ------------------------------------------------------------------------
    //! \brief Check if the viewport has keyboard focus.
    //! \return true if viewport is focused, false otherwise.
    // ------------------------------------------------------------------------
    bool isViewportFocused() const
    {
        return m_viewportFocused;
    }

    // ------------------------------------------------------------------------
    //! \brief Get the framebuffer texture ID.
    //! \return OpenGL texture ID for the framebuffer.
    // ------------------------------------------------------------------------
    GLuint getFramebufferTexture() const
    {
        return m_fboTexture;
    }

    // ------------------------------------------------------------------------
    //! \brief Get the framebuffer object ID.
    //! \return OpenGL framebuffer object ID.
    // ------------------------------------------------------------------------
    GLuint getFramebuffer() const
    {
        return m_fbo;
    }

private:

    // ------------------------------------------------------------------------
    //! \brief Setup ImGui style and colors.
    // ------------------------------------------------------------------------
    void setupStyle();

    // ------------------------------------------------------------------------
    //! \brief Setup and draw the main dockspace.
    // ------------------------------------------------------------------------
    void setupDockspace();

    // ------------------------------------------------------------------------
    //! \brief Draw the OpenGL viewport window.
    // ------------------------------------------------------------------------
    void drawViewport();

    // ------------------------------------------------------------------------
    //! \brief Draw the robot control panel.
    // ------------------------------------------------------------------------
    void drawRobotControlPanel();

    // ------------------------------------------------------------------------
    //! \brief Create the framebuffer for off-screen rendering.
    //! \param p_width Framebuffer width.
    //! \param p_height Framebuffer height.
    //! \return true if creation successful, false otherwise.
    // ------------------------------------------------------------------------
    bool createFramebuffer(int p_width, int p_height);

    // ------------------------------------------------------------------------
    //! \brief Delete the framebuffer and associated resources.
    // ------------------------------------------------------------------------
    void deleteFramebuffer();

    // ------------------------------------------------------------------------
    //! \brief Resize the framebuffer to new dimensions.
    //! \param p_width New framebuffer width.
    //! \param p_height New framebuffer height.
    // ------------------------------------------------------------------------
    void resizeFramebuffer(int p_width, int p_height);

private:

    // GLFW window (obtained from current context)
    GLFWwindow* m_window;

    // State flags
    bool m_initialized;
    bool m_dockingEnabled;
    bool m_viewportsEnabled;

    // Framebuffer for off-screen OpenGL rendering
    GLuint m_fbo;
    GLuint m_fboTexture;
    GLuint m_rbo;
    int m_fboWidth;
    int m_fboHeight;

    // Viewport state
    ImVec2 m_viewportSize;
    ImVec2 m_viewportPos;
    bool m_viewportHovered;
    bool m_viewportFocused;

    // Render callback for 3D content
    RenderCallback m_render_callback;

    // Robot management callbacks
    LoadRobotCallback m_load_robot_callback;
    RemoveRobotCallback m_remove_robot_callback;
    RobotListCallback m_robot_list_callback;
    GetJointsCallback m_get_joints_callback;
    SetJointCallback m_set_joint_callback;
    GetNodesCallback m_get_nodes_callback;
    SetControlModeCallback m_set_control_mode_callback;
    GetControlModeCallback m_get_control_mode_callback;
    SetEndEffectorCallback m_set_end_effector_callback;
    GetEndEffectorCallback m_get_end_effector_callback;
    SetCameraTargetCallback m_set_camera_target_callback;
    GetCameraTargetCallback m_get_camera_target_callback;

    // UI state
    std::string m_selected_robot;
    char m_urdf_path_buffer[512];
};

} // namespace robotik::viewer
