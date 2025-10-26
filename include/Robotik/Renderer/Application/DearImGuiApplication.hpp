/**
 * @file DearImGuiApplication.hpp
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
#include <utility>

namespace robotik::renderer::application
{

// ****************************************************************************
//! \brief Dear ImGui application wrapper providing docking and viewport
//! integration:
//! - Docking support for detachable windows.
//! - Framebuffer rendering for OpenGL content in ImGui widget.
//! - Customizable rendering callback for building the UI.
//! - Owned by OpenGLApplication.
// ****************************************************************************
class DearImGuiApplication
{
public:

    using RenderCallback = std::function<void()>;
    using MenuBarCallback = std::function<void()>;
    using MainPanelCallback = std::function<void()>;
    using SidePanelCallback = std::function<void()>;
    using StatusBarCallback = std::function<void()>;

    // ------------------------------------------------------------------------
    //! \brief Constructor.
    //! \param p_width Initial framebuffer width.
    //! \param p_height Initial framebuffer height.
    // ------------------------------------------------------------------------
    explicit DearImGuiApplication(size_t const p_width, size_t const p_height);

    // ------------------------------------------------------------------------
    //! \brief Destructor.
    // ------------------------------------------------------------------------
    ~DearImGuiApplication();

    // ------------------------------------------------------------------------
    //! \brief Set the render callback for 3D scene rendering.
    //! \param p_callback Function to call for rendering 3D content.
    // ------------------------------------------------------------------------
    void setRenderCallback(RenderCallback&& p_callback)
    {
        m_render_callback = std::move(p_callback);
    }

    // ------------------------------------------------------------------------
    //! \brief Set the menu bar callback.
    //! \param p_callback Function to call for menu bar rendering.
    // ------------------------------------------------------------------------
    void setMenuBarCallback(MenuBarCallback&& p_callback)
    {
        m_menu_bar_callback = std::move(p_callback);
    }

    // ------------------------------------------------------------------------
    //! \brief Set the main panel callback.
    //! \param p_callback Function to call for main panel rendering.
    // ------------------------------------------------------------------------
    void setMainPanelCallback(MainPanelCallback&& p_callback)
    {
        m_main_panel_callback = std::move(p_callback);
    }

    // ------------------------------------------------------------------------
    //! \brief Set the side panel callback.
    //! \param p_callback Function to call for side panel rendering.
    // ------------------------------------------------------------------------
    void setSidePanelCallback(SidePanelCallback&& p_callback)
    {
        m_side_panel_callback = std::move(p_callback);
    }

    // ------------------------------------------------------------------------
    //! \brief Set the status bar callback.
    //! \param p_callback Function to call for status bar rendering.
    // ------------------------------------------------------------------------
    void setStatusBarCallback(StatusBarCallback&& p_callback)
    {
        m_status_bar_callback = std::move(p_callback);
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
        return m_viewport_size;
    }

    // ------------------------------------------------------------------------
    //! \brief Get the current viewport position.
    //! \return Viewport position in screen coordinates.
    // ------------------------------------------------------------------------
    ImVec2 getViewportPos() const
    {
        return m_viewport_pos;
    }

    // ------------------------------------------------------------------------
    //! \brief Check if the viewport is hovered by the mouse.
    //! \return true if viewport is hovered, false otherwise.
    // ------------------------------------------------------------------------
    bool isViewportHovered() const
    {
        return m_viewport_hovered;
    }

    // ------------------------------------------------------------------------
    //! \brief Check if the viewport has keyboard focus.
    //! \return true if viewport is focused, false otherwise.
    // ------------------------------------------------------------------------
    bool isViewportFocused() const
    {
        return m_viewport_focused;
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
    GLFWwindow* m_window = nullptr;

    // State flags
    bool m_initialized = false;
    bool m_docking_enabled = true;
    bool m_viewports_enabled = true;

    // Framebuffer for off-screen OpenGL rendering
    GLuint m_fbo = 0;
    GLuint m_fbo_texture = 0;
    GLuint m_rbo = 0;
    int m_fbo_width = 0;
    int m_fbo_height = 0;

    // Viewport state
    ImVec2 m_viewport_size;
    ImVec2 m_viewport_pos;
    bool m_viewport_hovered = false;
    bool m_viewport_focused = false;

    // Render callback for 3D content
    RenderCallback m_render_callback;

    // ImGui UI callbacks
    MenuBarCallback m_menu_bar_callback;
    MainPanelCallback m_main_panel_callback;
    SidePanelCallback m_side_panel_callback;
    StatusBarCallback m_status_bar_callback;
};

} // namespace robotik::renderer::application
