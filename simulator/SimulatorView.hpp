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
#include "Compages/Renderer/Assets/CanvasTexture.hpp"
#include "Compages/Renderer/Scene.hpp"

#include <filesystem>

#include <cstdint>
#include <filesystem>
#include <memory>
#include <vector>

// Offscreen picture shown in an ImGui panel.
/// @brief The entity casts no shadow (a floor, a ground picture).
void noShadow(compages::world::Entity p_entity);

/// @brief A grey floor picture with a grid (a line every 10 cm on 3 m),
/// written once into the temporary directory; its path.
[[nodiscard]] std::filesystem::path floorGrid();

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

    /// @p_overlays: entities only the operator sees (teach axes, waypoint
    /// markers), hidden while this camera renders.
    RenderedCamera(compages::renderer::Scene& p_scene,
                   robotik::Camera const& p_camera,
                   compages::world::Entity p_link,
                   std::vector<compages::world::EntityId> const& p_overlays);

    bool capture(robotik::Camera const& p_camera,
                 robotik::CameraFrame& p_frame) override;
    void sync(robotik::Camera const& p_camera);

    [[nodiscard]] RenderTarget const& target() const
    {
        return m_target;
    }

private:

    compages::renderer::Scene& m_scene;
    std::vector<compages::world::EntityId> const& m_overlays;
    compages::world::Entity m_entity;
    RenderTarget m_target;
};

// Meshes and cameras of the simulator: everything Robotik does not render.
class SimulatorView final: public robotik::SceneView
{
public:

    explicit SimulatorView(compages::renderer::Scene& p_scene)
        : m_scene(p_scene)
    {
    }

    compages::world::Entity robot(compages::world::World& p_world,
                                  std::filesystem::path const& p_urdf) override;
    compages::world::Entity model(compages::world::World& p_world,
                                  std::filesystem::path const& p_urdf) override;
    void object(compages::world::Entity p_entity,
                robotik::ecs::Shape const& p_object) override;
    robotik::FrameSource* camera(robotik::Camera& p_camera,
                                 compages::world::Entity p_link) override;
    void ground(robotik::Image const& p_image,
                double p_width,
                double p_height) override;
    bool openCanvas(compages::world::Entity p_parent,
                    double p_width,
                    double p_height) override;
    void ink(double p_y,
             double p_z,
             float p_red,
             float p_green,
             float p_blue) override;

    /// @brief Marks @p_entity as a help for the operator: the 3D view shows
    /// it, the robot cameras never see it (it would fool perception).
    void overlay(compages::world::Entity p_entity)
    {
        m_overlays.push_back(p_entity.id());
    }

    [[nodiscard]] bool hasGround() const override
    {
        return m_grounded;
    }

    [[nodiscard]] std::vector<std::unique_ptr<RenderedCamera>> const&
    cameras() const
    {
        return m_cameras;
    }

    // Last frame of robot camera @p_index, uploaded for the ImGui panel.
    bool showFrame(std::size_t p_index, robotik::CameraFrame const& p_frame);

    // Picture of robot camera @p_index uploaded by showFrame.
    [[nodiscard]] RenderTarget const& frameTarget(std::size_t p_index);

private:

    compages::renderer::Scene& m_scene;
    std::vector<std::unique_ptr<RenderedCamera>> m_cameras;
    std::vector<std::unique_ptr<RenderTarget>> m_frames;
    std::vector<compages::world::EntityId> m_overlays;
    bool m_grounded = false;
    std::filesystem::path m_ground_file;
    compages::renderer::CanvasTexture m_paint;
    compages::world::Entity m_sheet;
    double m_sheet_width = 1.0;
    double m_sheet_height = 1.0;
    bool m_sheet_ready = false;
};
