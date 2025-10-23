/**
 * @file DearImGuiApp.hpp
 * @brief Dear ImGui application base class for the viewer.
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

namespace robotik::viewer
{
class OpenGLWindow; // Forward declaration
}

class ImGuiApp
{
public:

    using RenderCallback = std::function<void()>;

    ImGuiApp(robotik::viewer::OpenGLWindow* fenetreGL);
    ~ImGuiApp();

    // Set the render callback for 3D scene
    void setRenderCallback(RenderCallback callback)
    {
        m_render_callback = callback;
    }

    // Méthodes principales
    bool setup();
    void teardown();
    void draw();
    ImGuiIO& io();

    // Configuration
    void enableDocking(bool enable = true);
    void enableViewports(bool enable = true);

    // Accesseurs pour la vue OpenGL
    ImVec2 getViewportSize() const
    {
        return m_viewportSize;
    }
    ImVec2 getViewportPos() const
    {
        return m_viewportPos;
    }
    bool isViewportHovered() const
    {
        return m_viewportHovered;
    }
    bool isViewportFocused() const
    {
        return m_viewportFocused;
    }

    // Accesseurs de texture pour le rendu OpenGL
    GLuint getFramebufferTexture() const
    {
        return m_fboTexture;
    }
    GLuint getFramebuffer() const
    {
        return m_fbo;
    }

private:

    robotik::viewer::OpenGLWindow* m_fenetreGL;
    GLFWwindow* m_window;

    bool m_initialized;
    bool m_dockingEnabled;
    bool m_viewportsEnabled;

    // Framebuffer pour le rendu OpenGL
    GLuint m_fbo;
    GLuint m_fboTexture;
    GLuint m_rbo;
    int m_fboWidth;
    int m_fboHeight;

    // État de la viewport
    ImVec2 m_viewportSize;
    ImVec2 m_viewportPos;
    bool m_viewportHovered;
    bool m_viewportFocused;

    // Helpers
    void setupStyle();
    void setupDockspace();
    void drawViewport();
    bool createFramebuffer(int width, int height);
    void deleteFramebuffer();
    void resizeFramebuffer(int width, int height);

    // Render callback
    RenderCallback m_render_callback;
};
