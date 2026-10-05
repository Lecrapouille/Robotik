// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#pragma once

#include "Robotik/Robot/Robot.hpp"
#include "Robotik/Sensors/Camera.hpp"

#include "Compages/GPU/Framebuffer.hpp"
#include "Compages/GPU/Texture.hpp"
#include "Compages/Renderer/Scene.hpp"

#include <cstdint>
#include <memory>
#include <vector>

// Offscreen picture shown in an ImGui panel.
struct RenderTarget
{
    compages::gpu::Texture color;
    compages::gpu::Texture depth;
    compages::gpu::Framebuffer framebuffer;
    std::uint32_t width = 0;
    std::uint32_t height = 0;

    bool resize(std::uint32_t p_width, std::uint32_t p_height);
};

// A robot camera rendered by Compages: the image source of a Robotik camera.
class RenderedCamera final: public robotik::FrameSource
{
public:

    RenderedCamera(compages::renderer::Scene& p_scene,
                   robotik::Camera const& p_camera,
                   compages::world::Entity p_link);

    bool capture(robotik::Camera const& p_camera,
                 robotik::CameraFrame& p_frame) override;

    [[nodiscard]] RenderTarget const& target() const
    {
        return m_target;
    }

private:

    compages::renderer::Scene& m_scene;
    compages::world::Entity m_entity;
    RenderTarget m_target;
};

// Meshes and cameras of the simulator: everything Robotik does not render.
class SimulatorView final: public robotik::SceneView
{
public:

    explicit SimulatorView(compages::renderer::Scene& p_scene) : m_scene(p_scene)
    {
    }

    compages::world::Entity robot(compages::world::World& p_world,
                                  std::filesystem::path const& p_urdf) override;
    void object(compages::world::Entity p_entity,
                robotik::ecs::SceneObject const& p_object) override;
    robotik::FrameSource* camera(robotik::Camera& p_camera,
                                 compages::world::Entity p_link) override;

    [[nodiscard]] std::vector<std::unique_ptr<RenderedCamera>> const& cameras() const
    {
        return m_cameras;
    }

private:

    compages::renderer::Scene& m_scene;
    std::vector<std::unique_ptr<RenderedCamera>> m_cameras;
};
