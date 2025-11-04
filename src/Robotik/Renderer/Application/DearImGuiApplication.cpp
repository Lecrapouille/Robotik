/**
 * @file DearImGuiApplication.cpp
 * @brief Dear ImGui application wrapper with docking support.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#include "Robotik/Renderer/Application/DearImGuiApplication.hpp"

namespace robotik::renderer
{

// ----------------------------------------------------------------------------
DearImGuiApplication::DearImGuiApplication(size_t const p_width,
                                           size_t const p_height)
    : m_fbo_width(static_cast<int>(p_width)),
      m_fbo_height(static_cast<int>(p_height)),
      m_viewport_size(static_cast<float>(p_width),
                      static_cast<float>(p_height)),
      m_viewport_pos(0.0f, 0.0f)
{
}

// ----------------------------------------------------------------------------
DearImGuiApplication::~DearImGuiApplication()
{
    if (m_initialized)
    {
        teardown();
    }
}

// ----------------------------------------------------------------------------
bool DearImGuiApplication::setup()
{
    if (m_initialized)
    {
        return true;
    }

    // Get GLFW window from current context
    m_window = glfwGetCurrentContext();
    if (!m_window)
    {
        return false;
    }

    // Create ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();

    // Configure ImGui flags
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;

    if (m_docking_enabled)
    {
        io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
    }

    if (m_viewports_enabled)
    {
        io.ConfigFlags |= ImGuiConfigFlags_ViewportsEnable;
    }

    // Setup ImGui style
    setupStyle();

    // Initialize backends
    const char* glsl_version = "#version 330";
    if (!ImGui_ImplGlfw_InitForOpenGL(m_window, true))
    {
        return false;
    }

    if (!ImGui_ImplOpenGL3_Init(glsl_version))
    {
        ImGui_ImplGlfw_Shutdown();
        return false;
    }

    // Create framebuffer for OpenGL rendering
    if (!createFramebuffer(m_fbo_width, m_fbo_height))
    {
        teardown();
        return false;
    }

    m_initialized = true;
    return true;
}

// ----------------------------------------------------------------------------
void DearImGuiApplication::teardown()
{
    if (!m_initialized)
    {
        return;
    }

    deleteFramebuffer();

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    m_initialized = false;
    m_window = nullptr;
}

// ----------------------------------------------------------------------------
void DearImGuiApplication::draw()
{
    if (!m_initialized)
    {
        return;
    }

    // Start ImGui frame
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    // Setup dockspace
    if (m_docking_enabled)
    {
        setupDockspace();
    }

    // Draw OpenGL viewport
    drawViewport();

    // Draw user-defined panels
    if (m_main_panel_callback)
    {
        m_main_panel_callback();
    }

    if (m_side_panel_callback)
    {
        m_side_panel_callback();
    }

    if (m_status_bar_callback)
    {
        m_status_bar_callback();
    }

    // Finalize ImGui frame
    ImGui::Render();

    // Render to main window
    int display_w, display_h;
    glfwGetFramebufferSize(m_window, &display_w, &display_h);
    glViewport(0, 0, display_w, display_h);
    glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);

    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    // Update multi-viewports
    if (m_viewports_enabled)
    {
        ImGuiIO const& io = ImGui::GetIO();
        if (io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable)
        {
            GLFWwindow* backup_current_context = glfwGetCurrentContext();
            ImGui::UpdatePlatformWindows();
            ImGui::RenderPlatformWindowsDefault();
            glfwMakeContextCurrent(backup_current_context);
        }
    }
}

// ----------------------------------------------------------------------------
ImGuiIO& DearImGuiApplication::io()
{
    return ImGui::GetIO();
}

// ----------------------------------------------------------------------------
void DearImGuiApplication::enableDocking(bool p_enable)
{
    m_docking_enabled = p_enable;
    if (m_initialized)
    {
        ImGuiIO& io = ImGui::GetIO();
        if (p_enable)
        {
            io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
        }
        else
        {
            io.ConfigFlags &= ~ImGuiConfigFlags_DockingEnable;
        }
    }
}

// ----------------------------------------------------------------------------
void DearImGuiApplication::enableViewports(bool p_enable)
{
    m_viewports_enabled = p_enable;
    if (m_initialized)
    {
        ImGuiIO& io = ImGui::GetIO();
        if (p_enable)
        {
            io.ConfigFlags |= ImGuiConfigFlags_ViewportsEnable;
        }
        else
        {
            io.ConfigFlags &= ~ImGuiConfigFlags_ViewportsEnable;
        }
    }
}

// ----------------------------------------------------------------------------
void DearImGuiApplication::setupStyle() const
{
    ImGui::StyleColorsDark();

    ImGuiStyle& style = ImGui::GetStyle();

    if (ImGui::GetIO().ConfigFlags & ImGuiConfigFlags_ViewportsEnable)
    {
        style.WindowRounding = 0.0f;
        style.Colors[ImGuiCol_WindowBg].w = 1.0f;
    }
}

// ----------------------------------------------------------------------------
void DearImGuiApplication::setupDockspace() const
{
    static ImGuiDockNodeFlags dockspace_flags = ImGuiDockNodeFlags_None;

    ImGuiWindowFlags window_flags = ImGuiWindowFlags_NoDocking;

    const ImGuiViewport* viewport = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(viewport->WorkPos);
    ImGui::SetNextWindowSize(viewport->WorkSize);
    ImGui::SetNextWindowViewport(viewport->ID);

    window_flags |= ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoCollapse;
    window_flags |= ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove;
    window_flags |=
        ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoNavFocus;
    window_flags |= ImGuiWindowFlags_MenuBar;

    if (dockspace_flags & ImGuiDockNodeFlags_PassthruCentralNode)
    {
        window_flags |= ImGuiWindowFlags_NoBackground;
    }

    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0f, 0.0f));

    ImGui::Begin("DockSpace", nullptr, window_flags);
    ImGui::PopStyleVar(3);

    // Menu bar - call user callback if provided
    if (ImGui::BeginMenuBar())
    {
        if (m_menu_bar_callback)
        {
            m_menu_bar_callback();
        }
        ImGui::EndMenuBar();
    }

    ImGuiID dockspace_id = ImGui::GetID("MyDockSpace");
    ImGui::DockSpace(dockspace_id, ImVec2(0.0f, 0.0f), dockspace_flags);

    ImGui::End();
}

// ----------------------------------------------------------------------------
void DearImGuiApplication::drawViewport()
{
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
    ImGui::Begin("OpenGL Viewport");

    // Get window state
    m_viewport_hovered = ImGui::IsWindowHovered();
    m_viewport_focused = ImGui::IsWindowFocused();

    // Get available size
    ImVec2 viewport_panel_size = ImGui::GetContentRegionAvail();
    m_viewport_pos = ImGui::GetCursorScreenPos();

    // Resize framebuffer if necessary
    if (viewport_panel_size.x != m_viewport_size.x ||
        viewport_panel_size.y != m_viewport_size.y)
    {
        m_viewport_size = viewport_panel_size;
        if (m_viewport_size.x > 0 && m_viewport_size.y > 0)
        {
            resizeFramebuffer(static_cast<int>(m_viewport_size.x),
                              static_cast<int>(m_viewport_size.y));
        }
    }

    // Render OpenGL content to framebuffer
    if (m_fbo && m_viewport_size.x > 0 && m_viewport_size.y > 0)
    {
        glBindFramebuffer(GL_FRAMEBUFFER, m_fbo);
        glViewport(0,
                   0,
                   static_cast<int>(m_viewport_size.x),
                   static_cast<int>(m_viewport_size.y));

        // Call render callback for 3D content
        if (m_render_callback)
        {
            m_render_callback();
        }

        glBindFramebuffer(GL_FRAMEBUFFER, 0);

        // Display texture in ImGui
        ImGui::Image(
            reinterpret_cast<void*>(static_cast<intptr_t>(m_fbo_texture)),
            m_viewport_size,
            ImVec2(0, 1), // Flipped UV coords for OpenGL
            ImVec2(1, 0));
    }

    ImGui::End();
    ImGui::PopStyleVar();
}

// ----------------------------------------------------------------------------
bool DearImGuiApplication::createFramebuffer(int p_width, int p_height)
{
    // Generate framebuffer
    glGenFramebuffers(1, &m_fbo);
    glBindFramebuffer(GL_FRAMEBUFFER, m_fbo);

    // Create color texture
    glGenTextures(1, &m_fbo_texture);
    glBindTexture(GL_TEXTURE_2D, m_fbo_texture);
    glTexImage2D(GL_TEXTURE_2D,
                 0,
                 GL_RGB,
                 p_width,
                 p_height,
                 0,
                 GL_RGB,
                 GL_UNSIGNED_BYTE,
                 nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glFramebufferTexture2D(
        GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, m_fbo_texture, 0);

    // Create renderbuffer for depth/stencil
    glGenRenderbuffers(1, &m_rbo);
    glBindRenderbuffer(GL_RENDERBUFFER, m_rbo);
    glRenderbufferStorage(
        GL_RENDERBUFFER, GL_DEPTH24_STENCIL8, p_width, p_height);
    glFramebufferRenderbuffer(
        GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, m_rbo);

    // Check framebuffer status
    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
    {
        deleteFramebuffer();
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        return false;
    }

    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    m_fbo_width = p_width;
    m_fbo_height = p_height;

    return true;
}

// ----------------------------------------------------------------------------
void DearImGuiApplication::deleteFramebuffer()
{
    if (m_fbo)
    {
        glDeleteFramebuffers(1, &m_fbo);
        m_fbo = 0;
    }
    if (m_fbo_texture)
    {
        glDeleteTextures(1, &m_fbo_texture);
        m_fbo_texture = 0;
    }
    if (m_rbo)
    {
        glDeleteRenderbuffers(1, &m_rbo);
        m_rbo = 0;
    }
}

// ----------------------------------------------------------------------------
void DearImGuiApplication::resizeFramebuffer(int p_width, int p_height)
{
    if (p_width <= 0 || p_height <= 0)
    {
        return;
    }

    deleteFramebuffer();
    createFramebuffer(p_width, p_height);
}

} // namespace robotik::renderer