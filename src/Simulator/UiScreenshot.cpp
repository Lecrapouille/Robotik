// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file UiScreenshot.cpp
/// --screenshot-ui: a PNG of the whole window, panels included, to check
/// the interface without a person in front of it.

#include "MainLoop.hpp"

#include <GLFW/glfw3.h>

#include <cstddef>
#include <vector>

// The writer is compiled here, private to this file (Compages has its own).
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#pragma GCC diagnostic ignored "-Wold-style-cast"
#pragma GCC diagnostic ignored "-Wcast-qual"
#pragma GCC diagnostic ignored "-Wswitch-default"
#pragma GCC diagnostic ignored "-Wuseless-cast"
#pragma GCC diagnostic ignored "-Wdouble-promotion"
#pragma GCC diagnostic ignored "-Wfloat-conversion"
#pragma GCC diagnostic ignored "-Wunused-function"
#define STB_IMAGE_WRITE_STATIC
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <stb_image_write.h>
#pragma GCC diagnostic pop

/// glReadPixels, found through GLFW: the simulator does not link OpenGL.
using ReadPixels = void (*)(int, int, int, int, unsigned, unsigned, void*);

bool saveWindowPng(std::filesystem::path const& p_path, int p_width, int p_height)
{
#if defined(__EMSCRIPTEN__)
    (void)p_path;
    (void)p_width;
    (void)p_height;
    return false;
#else
    constexpr unsigned GL_RGBA_ = 0x1908U;
    constexpr unsigned GL_UNSIGNED_BYTE_ = 0x1401U;
    auto const read =
        reinterpret_cast<ReadPixels>(glfwGetProcAddress("glReadPixels"));
    if (read == nullptr || p_width <= 0 || p_height <= 0)
    {
        return false;
    }
    std::vector<std::byte> pixels(static_cast<std::size_t>(p_width) *
                                  static_cast<std::size_t>(p_height) * 4U);
    read(0, 0, p_width, p_height, GL_RGBA_, GL_UNSIGNED_BYTE_, pixels.data());
    // OpenGL gives the bottom row first.
    stbi_flip_vertically_on_write(1);
    return stbi_write_png(p_path.string().c_str(), p_width, p_height, 4,
                          pixels.data(), p_width * 4) != 0;
#endif
}
