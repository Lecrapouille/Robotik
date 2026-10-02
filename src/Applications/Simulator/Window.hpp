#pragma once

struct GLFWwindow;

// ****************************************************************************
//! @brief GLFW window, OpenGL 4.5 via Compages GPU, and Dear ImGui (docking).
// ****************************************************************************
struct Window
{
    GLFWwindow* handle = nullptr;
    bool glfw_ready = false;
    bool device_ready = false;
    bool imgui_ready = false;

    bool open();
    ~Window();
};
