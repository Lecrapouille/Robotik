// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "Screenshot.hpp"

#include "SimulatorView.hpp"

#include "Compages/GPU/RenderPass.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#pragma GCC diagnostic ignored "-Wcast-qual"
#pragma GCC diagnostic ignored "-Wdouble-promotion"
#pragma GCC diagnostic ignored "-Wfloat-conversion"
#pragma GCC diagnostic ignored "-Wuseless-cast"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wold-style-cast"
#pragma GCC diagnostic ignored "-Wimplicit-fallthrough"
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#define STB_IMAGE_WRITE_STATIC
#include "stb_image_write.h"
#pragma GCC diagnostic pop

std::string saveScreenshot(RenderTarget const& p_target,
                           std::filesystem::path const& p_path)
{
    if (p_target.width == 0U || p_target.height == 0U)
    {
        return "the 3D view is not drawn yet";
    }
    // A pass names the target to read; it clears nothing.
    compages::gpu::RenderPass const pass(
        p_target.framebuffer,
        compages::gpu::PassDesc{ .clear_color = false, .clear_depth = false });
    compages::Result<std::vector<std::byte>> const pixels =
        compages::gpu::readPixels();
    if (!pixels)
    {
        return "cannot read the 3D view: " + pixels.error();
    }
    // The device gives the bottom row first, a PNG holds the top row first.
    stbi_flip_vertically_on_write(1);
    int const written = stbi_write_png(p_path.string().c_str(),
                                       static_cast<int>(p_target.width),
                                       static_cast<int>(p_target.height),
                                       4,
                                       pixels.value().data(),
                                       static_cast<int>(p_target.width * 4U));
    return written != 0 ? std::string() : "cannot write " + p_path.string();
}
