/**
 * @file DearImGuiApp.cpp
 * @brief Dear ImGui application base class for the viewer.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#include "Robotik/Viewer/DearImGuiApp.hpp"
#include "Robotik/Viewer/OpenGLWindow.hpp"

ImGuiApp::ImGuiApp(robotik::viewer::OpenGLWindow* fenetreGL)
    : m_fenetreGL(fenetreGL),
      m_window(nullptr),
      m_initialized(false),
      m_dockingEnabled(true),
      m_viewportsEnabled(true),
      m_fbo(0),
      m_fboTexture(0),
      m_rbo(0),
      m_fboWidth(1280),
      m_fboHeight(720),
      m_viewportSize(1280, 720),
      m_viewportPos(0, 0),
      m_viewportHovered(false),
      m_viewportFocused(false)
{
}

ImGuiApp::~ImGuiApp()
{
    if (m_initialized)
    {
        teardown();
    }
}

bool ImGuiApp::setup()
{
    if (m_initialized)
    {
        return true;
    }

    // Récupération de la fenêtre GLFW
    m_window = glfwGetCurrentContext();
    if (!m_window)
    {
        return false;
    }

    // Création du contexte ImGui
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();

    // Configuration des flags
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;

    if (m_dockingEnabled)
    {
        io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
    }

    if (m_viewportsEnabled)
    {
        io.ConfigFlags |= ImGuiConfigFlags_ViewportsEnable;
    }

    // Configuration du style
    setupStyle();

    // Initialisation des backends
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

    // Création du framebuffer pour le rendu OpenGL
    if (!createFramebuffer(m_fboWidth, m_fboHeight))
    {
        teardown();
        return false;
    }

    m_initialized = true;
    return true;
}

void ImGuiApp::teardown()
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

void ImGuiApp::draw()
{
    if (!m_initialized)
    {
        return;
    }

    // Début de la frame ImGui
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    // Configuration du dockspace
    if (m_dockingEnabled)
    {
        setupDockspace();
    }

    // Rendu de la viewport OpenGL intégrée
    drawViewport();

    // Fin de la frame ImGui
    ImGui::Render();

    // Rendu final sur la fenêtre principale
    int display_w, display_h;
    glfwGetFramebufferSize(m_window, &display_w, &display_h);
    glViewport(0, 0, display_w, display_h);
    glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);

    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    // Mise à jour des viewports multi-fenêtres
    if (m_viewportsEnabled)
    {
        ImGuiIO& io = ImGui::GetIO();
        if (io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable)
        {
            GLFWwindow* backup_current_context = glfwGetCurrentContext();
            ImGui::UpdatePlatformWindows();
            ImGui::RenderPlatformWindowsDefault();
            glfwMakeContextCurrent(backup_current_context);
        }
    }
}

ImGuiIO& ImGuiApp::io()
{
    return ImGui::GetIO();
}

void ImGuiApp::enableDocking(bool enable)
{
    m_dockingEnabled = enable;
    if (m_initialized)
    {
        ImGuiIO& io = ImGui::GetIO();
        if (enable)
        {
            io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
        }
        else
        {
            io.ConfigFlags &= ~ImGuiConfigFlags_DockingEnable;
        }
    }
}

void ImGuiApp::enableViewports(bool enable)
{
    m_viewportsEnabled = enable;
    if (m_initialized)
    {
        ImGuiIO& io = ImGui::GetIO();
        if (enable)
        {
            io.ConfigFlags |= ImGuiConfigFlags_ViewportsEnable;
        }
        else
        {
            io.ConfigFlags &= ~ImGuiConfigFlags_ViewportsEnable;
        }
    }
}

void ImGuiApp::setupStyle()
{
    ImGui::StyleColorsDark();

    ImGuiStyle& style = ImGui::GetStyle();

    if (ImGui::GetIO().ConfigFlags & ImGuiConfigFlags_ViewportsEnable)
    {
        style.WindowRounding = 0.0f;
        style.Colors[ImGuiCol_WindowBg].w = 1.0f;
    }
}

void ImGuiApp::setupDockspace()
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

    // Menu bar optionnel
    if (ImGui::BeginMenuBar())
    {
        if (ImGui::BeginMenu("Fichier"))
        {
            if (ImGui::MenuItem("Quitter"))
            {
                glfwSetWindowShouldClose(m_window, GLFW_TRUE);
            }
            ImGui::EndMenu();
        }
        ImGui::EndMenuBar();
    }

    ImGuiID dockspace_id = ImGui::GetID("MyDockSpace");
    ImGui::DockSpace(dockspace_id, ImVec2(0.0f, 0.0f), dockspace_flags);

    ImGui::End();
}

void ImGuiApp::drawViewport()
{
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
    ImGui::Begin("Viewport OpenGL");

    // Récupération de l'état de la fenêtre
    m_viewportHovered = ImGui::IsWindowHovered();
    m_viewportFocused = ImGui::IsWindowFocused();

    // Récupération de la taille disponible
    ImVec2 viewportPanelSize = ImGui::GetContentRegionAvail();
    m_viewportPos = ImGui::GetCursorScreenPos();

    // Redimensionnement du framebuffer si nécessaire
    if (viewportPanelSize.x != m_viewportSize.x ||
        viewportPanelSize.y != m_viewportSize.y)
    {
        m_viewportSize = viewportPanelSize;
        if (m_viewportSize.x > 0 && m_viewportSize.y > 0)
        {
            resizeFramebuffer((int)m_viewportSize.x, (int)m_viewportSize.y);
        }
    }

    // Rendu OpenGL dans le framebuffer
    if (m_fbo && m_viewportSize.x > 0 && m_viewportSize.y > 0)
    {
        glBindFramebuffer(GL_FRAMEBUFFER, m_fbo);
        glViewport(0, 0, (int)m_viewportSize.x, (int)m_viewportSize.y);

        // Appel du callback de rendu 3D
        if (m_render_callback)
        {
            m_render_callback();
        }

        glBindFramebuffer(GL_FRAMEBUFFER, 0);

        // Affichage de la texture dans ImGui
        ImGui::Image((void*)(intptr_t)m_fboTexture,
                     m_viewportSize,
                     ImVec2(0, 1), // UV coords inversées pour OpenGL
                     ImVec2(1, 0));
    }

    ImGui::End();
    ImGui::PopStyleVar();
}

bool ImGuiApp::createFramebuffer(int width, int height)
{
    // Génération du framebuffer
    glGenFramebuffers(1, &m_fbo);
    glBindFramebuffer(GL_FRAMEBUFFER, m_fbo);

    // Création de la texture de couleur
    glGenTextures(1, &m_fboTexture);
    glBindTexture(GL_TEXTURE_2D, m_fboTexture);
    glTexImage2D(GL_TEXTURE_2D,
                 0,
                 GL_RGB,
                 width,
                 height,
                 0,
                 GL_RGB,
                 GL_UNSIGNED_BYTE,
                 nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glFramebufferTexture2D(
        GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, m_fboTexture, 0);

    // Création du renderbuffer pour depth/stencil
    glGenRenderbuffers(1, &m_rbo);
    glBindRenderbuffer(GL_RENDERBUFFER, m_rbo);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH24_STENCIL8, width, height);
    glFramebufferRenderbuffer(
        GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, m_rbo);

    // Vérification du framebuffer
    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
    {
        deleteFramebuffer();
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        return false;
    }

    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    m_fboWidth = width;
    m_fboHeight = height;

    return true;
}

void ImGuiApp::deleteFramebuffer()
{
    if (m_fbo)
    {
        glDeleteFramebuffers(1, &m_fbo);
        m_fbo = 0;
    }
    if (m_fboTexture)
    {
        glDeleteTextures(1, &m_fboTexture);
        m_fboTexture = 0;
    }
    if (m_rbo)
    {
        glDeleteRenderbuffers(1, &m_rbo);
        m_rbo = 0;
    }
}

void ImGuiApp::resizeFramebuffer(int width, int height)
{
    if (width <= 0 || height <= 0)
    {
        return;
    }

    deleteFramebuffer();
    createFramebuffer(width, height);
}