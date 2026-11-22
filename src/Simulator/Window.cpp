// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "Window.hpp"

#include "Compages/GPU/GPU.hpp"

#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

#include <implot.h>

#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <string>

#if defined(__EMSCRIPTEN__) && !defined(EXAEQUOS)
#    include <emscripten.h>
#    include <emscripten/html5.h>
#endif

/// imgui.ini lives in the user configuration, not in the folder the
/// simulator was started from.
static std::string settingsFile()
{
    std::filesystem::path base;
    if (char const* config = std::getenv("XDG_CONFIG_HOME");
        config != nullptr && *config != '\0')
    {
        base = config;
    }
    else if (char const* home = std::getenv("HOME");
             home != nullptr && *home != '\0')
    {
        base = std::filesystem::path(home) / ".config";
    }
    else
    {
        return "imgui.ini";
    }
    std::error_code error;
    std::filesystem::create_directories(base / "robotik", error);
    return (base / "robotik" / "simulator.ini").string();
}

/// Loads the interface fonts when the font directory is present.
static void loadFonts()
{
    ImGuiIO& io = ImGui::GetIO();
#ifdef ROBOTIK_FONTS_DIR
    std::filesystem::path const fonts(ROBOTIK_FONTS_DIR);
    std::filesystem::path const text = fonts / "Roboto-Medium.ttf";
    std::filesystem::path const mono = fonts / "Cousine-Regular.ttf";
    if (std::filesystem::exists(text) &&
        io.Fonts->AddFontFromFileTTF(text.string().c_str(), 14.0F) != nullptr)
    {
        // Second font, for the file viewer (see monoFont).
        if (std::filesystem::exists(mono))
        {
            io.Fonts->AddFontFromFileTTF(mono.string().c_str(), 13.0F);
        }
        return;
    }
#endif
    ImFontConfig fallback;
    fallback.SizePixels = 14.0F;
    io.Fonts->AddFontDefault(&fallback);
}

/// Color from 0xRRGGBB and an opacity.
static ImVec4 rgb(unsigned p_hex, float p_alpha = 1.0F)
{
    return ImVec4(static_cast<float>((p_hex >> 16U) & 0xFFU) / 255.0F,
                  static_cast<float>((p_hex >> 8U) & 0xFFU) / 255.0F,
                  static_cast<float>(p_hex & 0xFFU) / 255.0F,
                  p_alpha);
}

/// Dark theme: grey panels, one amber accent for what is chosen, few
/// borders. The status colors are in Colors.hpp.
static void applyStyle()
{
    ImGui::StyleColorsDark();
    ImGuiStyle& style = ImGui::GetStyle();
    style.FontSizeBase = 14.0F;
    style.WindowRounding = 0.0F;
    style.ChildRounding = 4.0F;
    style.FrameRounding = 5.0F;
    style.PopupRounding = 6.0F;
    style.GrabRounding = 3.0F;
    style.GrabMinSize = 8.0F;
    style.TabRounding = 4.0F;
    style.ScrollbarRounding = 6.0F;
    style.ScrollbarSize = 12.0F;
    style.WindowPadding = ImVec2(10.0F, 8.0F);
    style.FramePadding = ImVec2(8.0F, 3.0F);
    style.ItemSpacing = ImVec2(8.0F, 5.0F);
    style.ItemInnerSpacing = ImVec2(5.0F, 4.0F);
    style.CellPadding = ImVec2(6.0F, 3.0F);
    style.WindowBorderSize = 1.0F;
    style.ChildBorderSize = 1.0F;
    style.PopupBorderSize = 1.0F;
    style.FrameBorderSize = 1.0F;
    style.TabBarBorderSize = 1.0F;
    style.SeparatorTextBorderSize = 1.0F;
    style.DockingSeparatorSize = 4.0F;

    ImVec4 const background = rgb(0x141619);
    ImVec4 const panel = rgb(0x1C1F23);
    ImVec4 const raised = rgb(0x24282D);
    ImVec4 const hover = rgb(0x32373D);
    ImVec4 const border = rgb(0x33383E);
    ImVec4 const accent = rgb(0xEEB563);
    ImVec4* colors = style.Colors;
    colors[ImGuiCol_Text] = rgb(0xE3E6EA);
    colors[ImGuiCol_TextDisabled] = rgb(0x969FA9);
    colors[ImGuiCol_WindowBg] = panel;
    colors[ImGuiCol_ChildBg] = ImVec4(0.0F, 0.0F, 0.0F, 0.0F);
    colors[ImGuiCol_PopupBg] = rgb(0x1D2126, 0.99F);
    colors[ImGuiCol_Border] = border;
    colors[ImGuiCol_BorderShadow] = ImVec4(0.0F, 0.0F, 0.0F, 0.0F);
    colors[ImGuiCol_MenuBarBg] = rgb(0x1B1E22);
    colors[ImGuiCol_TitleBg] = rgb(0x1B1E22);
    colors[ImGuiCol_TitleBgActive] = rgb(0x1B1E22);
    colors[ImGuiCol_TitleBgCollapsed] = rgb(0x1B1E22);
    colors[ImGuiCol_FrameBg] = raised;
    colors[ImGuiCol_FrameBgHovered] = hover;
    colors[ImGuiCol_FrameBgActive] = hover;
    colors[ImGuiCol_Button] = raised;
    colors[ImGuiCol_ButtonHovered] = hover;
    colors[ImGuiCol_ButtonActive] = rgb(0xEEB563, 0.30F);
    colors[ImGuiCol_Header] = raised;
    colors[ImGuiCol_HeaderHovered] = hover;
    colors[ImGuiCol_HeaderActive] = rgb(0xEEB563, 0.16F);
    colors[ImGuiCol_Separator] = border;
    colors[ImGuiCol_SeparatorHovered] = accent;
    colors[ImGuiCol_SeparatorActive] = accent;
    colors[ImGuiCol_ResizeGrip] = ImVec4(0.0F, 0.0F, 0.0F, 0.0F);
    colors[ImGuiCol_ResizeGripHovered] = accent;
    colors[ImGuiCol_ResizeGripActive] = accent;
    colors[ImGuiCol_Tab] = panel;
    colors[ImGuiCol_TabHovered] = hover;
    colors[ImGuiCol_TabSelected] = raised;
    colors[ImGuiCol_TabSelectedOverline] = accent;
    colors[ImGuiCol_TabDimmed] = panel;
    colors[ImGuiCol_TabDimmedSelected] = raised;
    colors[ImGuiCol_TabDimmedSelectedOverline] = border;
    colors[ImGuiCol_CheckMark] = accent;
    // See-through: the value is written under the grab.
    colors[ImGuiCol_SliderGrab] = rgb(0xEEB563, 0.45F);
    colors[ImGuiCol_SliderGrabActive] = rgb(0xEEB563, 0.70F);
    colors[ImGuiCol_ScrollbarBg] = ImVec4(0.0F, 0.0F, 0.0F, 0.0F);
    colors[ImGuiCol_ScrollbarGrab] = border;
    colors[ImGuiCol_ScrollbarGrabHovered] = hover;
    colors[ImGuiCol_ScrollbarGrabActive] = rgb(0x59616B);
    colors[ImGuiCol_PlotHistogram] = accent;
    colors[ImGuiCol_TextSelectedBg] = rgb(0xEEB563, 0.30F);
    colors[ImGuiCol_NavCursor] = accent;
    colors[ImGuiCol_DockingPreview] = rgb(0xEEB563, 0.40F);
    colors[ImGuiCol_DockingEmptyBg] = background;
    colors[ImGuiCol_TableHeaderBg] = rgb(0x202328);
    colors[ImGuiCol_TableBorderStrong] = border;
    colors[ImGuiCol_TableBorderLight] = rgb(0x2A2E33);
    colors[ImGuiCol_TableRowBgAlt] = ImVec4(1.0F, 1.0F, 1.0F, 0.025F);
}

bool Window::open()
{
    if (glfwInit() == GLFW_FALSE)
    {
        return false;
    }
    glfw_ready = true;
#if defined(__EMSCRIPTEN__)
    // WebGL 2 (OpenGL ES 3), the GLES3 backend of Compages.
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
#    if defined(GLFW_SCALE_TO_MONITOR) && !defined(EXAEQUOS)
    glfwWindowHint(GLFW_SCALE_TO_MONITOR, GLFW_TRUE);
#    endif
    char const* glsl_version = "#version 300 es";
#else
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 5);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_DEPTH_BITS, 24);
    char const* glsl_version = "#version 450";
#endif
    handle = glfwCreateWindow(1600, 900, "RobotIK Simulator", nullptr, nullptr);
    if (handle == nullptr)
    {
        return false;
    }
    glfwMakeContextCurrent(handle);
    glfwSetWindowUserPointer(handle, this);
    // Before ImGui: the GLFW backend chains this callback for the wheel.
    glfwSetScrollCallback(handle, &Window::onScroll);
    glfwSwapInterval(1);

    // On the web, Compages is built with its GLES3 backend (WebGL 2), which
    // needs no loader: the function pointer is not read.
    auto ready = compages::gpu::init(
        reinterpret_cast<compages::gpu::LoadProc>(glfwGetProcAddress));
    if (!ready)
    {
        std::cerr << ready.error() << '\n';
#if defined(__EMSCRIPTEN__)
        // A browser without WebGL 2: the panels still open, without the
        // 3D view.
#else
        return false;
#endif
    }
    else
    {
        device_ready = true;
    }

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();
    static std::string const settings = settingsFile();
    ImGui::GetIO().IniFilename = settings.c_str();
    ImGui::GetIO().ConfigFlags |= ImGuiConfigFlags_DockingEnable;
    applyStyle();
    loadFonts();
    ImGui_ImplGlfw_InitForOpenGL(handle, true);
    ImGui_ImplOpenGL3_Init(glsl_version);
#if defined(__EMSCRIPTEN__) && !defined(EXAEQUOS)
    // Dear ImGui fixes the mouse wheel; the window follows the page.
    ImGui_ImplGlfw_InstallEmscriptenCallbacks(handle, "#canvas");
    emscripten_set_resize_callback(EMSCRIPTEN_EVENT_TARGET_WINDOW,
                                   handle,
                                   false,
                                   [](int /*event_type*/,
                                      EmscriptenUiEvent const* event,
                                      void* window) -> EM_BOOL
                                   {
                                       glfwSetWindowSize(
                                           static_cast<GLFWwindow*>(window),
                                           event->windowInnerWidth,
                                           event->windowInnerHeight);
                                       return EM_TRUE;
                                   });
    int const page_width = EM_ASM_INT({ return window.innerWidth; });
    int const page_height = EM_ASM_INT({ return window.innerHeight; });
    glfwSetWindowSize(handle, page_width, page_height);
#endif
    imgui_ready = true;
    return true;
}

void Window::beginInputFrame()
{
    m_scroll = 0.0F;
}

void Window::onScroll(GLFWwindow* p_window, double /*p_x*/, double p_y)
{
    auto* self = static_cast<Window*>(glfwGetWindowUserPointer(p_window));
    if (self != nullptr)
    {
        self->m_scroll += static_cast<float>(p_y);
    }
}

Window::~Window()
{
    if (imgui_ready)
    {
        ImGui_ImplOpenGL3_Shutdown();
        ImGui_ImplGlfw_Shutdown();
        ImPlot::DestroyContext();
        ImGui::DestroyContext();
    }
    if (device_ready)
    {
        compages::gpu::shutdown();
    }
    if (handle != nullptr)
    {
        glfwDestroyWindow(handle);
    }
    if (glfw_ready)
    {
        glfwTerminate();
    }
}
