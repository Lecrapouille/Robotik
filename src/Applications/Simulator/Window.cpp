#include "Window.hpp"

#include "Compages/GPU/GPU.hpp"

#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

#include <iostream>

//------------------------------------------------------------------------------
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
    ImGui::GetIO().ConfigFlags |= ImGuiConfigFlags_DockingEnable;
    ImGui::StyleColorsDark();
    ImGui_ImplGlfw_InitForOpenGL(handle, true);
    ImGui_ImplOpenGL3_Init("#version 450");
    imgui_ready = true;
    return true;
}

//------------------------------------------------------------------------------
Window::~Window()
{
    if (imgui_ready)
    {
        ImGui_ImplOpenGL3_Shutdown();
        ImGui_ImplGlfw_Shutdown();
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
