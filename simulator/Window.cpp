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
        io.Fonts->AddFontFromFileTTF(text.string().c_str(), 15.0F) != nullptr)
    {
#ifdef ROBOTIK_MATH_FONT
        // Symbols of GRAFCET receptivities and (max,+) algebra (↑ ↓ ⨁ ⊗ ≥),
        // missing from the text font.
        if (std::filesystem::exists(ROBOTIK_MATH_FONT))
        {
            ImFontConfig merge;
            merge.MergeMode = true;
            io.Fonts->AddFontFromFileTTF(ROBOTIK_MATH_FONT, 15.0F, &merge);
        }
#endif
        // Second font, for the file viewer (see monoFont).
        if (std::filesystem::exists(mono))
        {
            io.Fonts->AddFontFromFileTTF(mono.string().c_str(), 14.0F);
        }
        return;
    }
#endif
    io.Fonts->AddFontDefault();
}

/// Dark theme and the rounding used by every panel.
static void applyStyle()
{
    ImGui::StyleColorsDark();
    ImGuiStyle& style = ImGui::GetStyle();
    style.WindowRounding = 4.0F;
    style.ChildRounding = 4.0F;
    style.FrameRounding = 4.0F;
    style.PopupRounding = 4.0F;
    style.GrabRounding = 3.0F;
    style.TabRounding = 4.0F;
    style.ScrollbarRounding = 6.0F;
    style.WindowPadding = ImVec2(10.0F, 8.0F);
    style.FramePadding = ImVec2(8.0F, 4.0F);
    style.ItemSpacing = ImVec2(8.0F, 6.0F);
    style.CellPadding = ImVec2(6.0F, 3.0F);
    style.WindowBorderSize = 0.0F;
    style.TabBarBorderSize = 1.0F;
    style.SeparatorTextBorderSize = 1.0F;
    ImVec4* colors = style.Colors;
    ImVec4 const panel(0.11F, 0.12F, 0.14F, 1.0F);
    ImVec4 const accent(0.26F, 0.48F, 0.80F, 1.0F);
    colors[ImGuiCol_WindowBg] = panel;
    colors[ImGuiCol_ChildBg] = panel;
    colors[ImGuiCol_PopupBg] = ImVec4(0.13F, 0.14F, 0.17F, 0.98F);
    colors[ImGuiCol_MenuBarBg] = ImVec4(0.09F, 0.10F, 0.12F, 1.0F);
    colors[ImGuiCol_TitleBg] = ImVec4(0.09F, 0.10F, 0.12F, 1.0F);
    colors[ImGuiCol_TitleBgActive] = ImVec4(0.09F, 0.10F, 0.12F, 1.0F);
    colors[ImGuiCol_FrameBg] = ImVec4(0.18F, 0.20F, 0.24F, 1.0F);
    colors[ImGuiCol_FrameBgHovered] = ImVec4(0.23F, 0.26F, 0.31F, 1.0F);
    colors[ImGuiCol_FrameBgActive] = ImVec4(0.27F, 0.31F, 0.37F, 1.0F);
    colors[ImGuiCol_Button] = ImVec4(0.20F, 0.23F, 0.28F, 1.0F);
    colors[ImGuiCol_ButtonHovered] = ImVec4(0.26F, 0.31F, 0.38F, 1.0F);
    colors[ImGuiCol_ButtonActive] = accent;
    colors[ImGuiCol_Header] = ImVec4(0.20F, 0.23F, 0.28F, 1.0F);
    colors[ImGuiCol_HeaderHovered] = ImVec4(0.26F, 0.31F, 0.38F, 1.0F);
    colors[ImGuiCol_HeaderActive] = accent;
    colors[ImGuiCol_Tab] = ImVec4(0.11F, 0.12F, 0.14F, 1.0F);
    colors[ImGuiCol_TabHovered] = ImVec4(0.26F, 0.31F, 0.38F, 1.0F);
    colors[ImGuiCol_TabSelected] = ImVec4(0.18F, 0.21F, 0.26F, 1.0F);
    colors[ImGuiCol_TabSelectedOverline] = accent;
    colors[ImGuiCol_TabDimmed] = ImVec4(0.09F, 0.10F, 0.12F, 1.0F);
    colors[ImGuiCol_TabDimmedSelected] = ImVec4(0.14F, 0.16F, 0.19F, 1.0F);
    colors[ImGuiCol_CheckMark] = accent;
    colors[ImGuiCol_SliderGrab] = accent;
    colors[ImGuiCol_SliderGrabActive] = ImVec4(0.36F, 0.58F, 0.90F, 1.0F);
    colors[ImGuiCol_DockingPreview] = ImVec4(0.26F, 0.48F, 0.80F, 0.5F);
    colors[ImGuiCol_DockingEmptyBg] = ImVec4(0.07F, 0.08F, 0.09F, 1.0F);
    colors[ImGuiCol_TableHeaderBg] = ImVec4(0.15F, 0.17F, 0.20F, 1.0F);
    colors[ImGuiCol_TableRowBgAlt] = ImVec4(1.0F, 1.0F, 1.0F, 0.025F);
}

bool Window::open()
{
    if (glfwInit() == GLFW_FALSE)
    {
        return false;
    }
    glfw_ready = true;
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 5);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_DEPTH_BITS, 24);
    handle = glfwCreateWindow(1600, 900, "Robotik Simulator", nullptr, nullptr);
    if (handle == nullptr)
    {
        return false;
    }
    glfwMakeContextCurrent(handle);
    glfwSetWindowUserPointer(handle, this);
    // Before ImGui: the GLFW backend chains this callback for the wheel.
    glfwSetScrollCallback(handle, &Window::onScroll);
    glfwSwapInterval(1);

    auto ready = compages::gpu::init(
        reinterpret_cast<compages::gpu::LoadProc>(glfwGetProcAddress));
    if (!ready)
    {
        std::cerr << ready.error() << '\n';
        return false;
    }
    device_ready = true;

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();
    static std::string const settings = settingsFile();
    ImGui::GetIO().IniFilename = settings.c_str();
    ImGui::GetIO().ConfigFlags |= ImGuiConfigFlags_DockingEnable;
    applyStyle();
    loadFonts();
    ImGui_ImplGlfw_InitForOpenGL(handle, true);
    ImGui_ImplOpenGL3_Init("#version 450");
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
