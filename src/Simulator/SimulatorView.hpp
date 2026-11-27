// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#pragma once

#include "RobotIK/Robot/SceneView.hpp"
#include "RobotIK/Robot/Robot.hpp"
#include "RobotIK/Robot/Sensors/Camera.hpp"

#include "Compages/GPU/Framebuffer.hpp"
#include "Compages/GPU/Texture.hpp"
#include "Compages/Renderer/Components/MeshRenderer.hpp"
#include "Compages/Renderer/Procedural/CanvasTexture.hpp"
#include "Compages/Renderer/Render/RenderTarget.hpp"
#include "Compages/Renderer/Scene/Scene.hpp"

#include <filesystem>

#include <cstdint>
#include <filesystem>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

// Offscreen picture shown in an ImGui panel.
/// The entity casts no shadow (a floor, a ground picture).
void noShadow(compages::world::Entity p_entity);

/// A grey floor picture with a grid (a line every 10 cm on 3 m),
/// written once into the temporary directory; its path.
[[nodiscard]] std::filesystem::path floorGrid();

/// A picture the Scene draws into (colour, depth): the 3D view, a preview,
/// the eye of a robot camera.
using RenderTarget = compages::renderer::RenderTarget;

/// A picture written from the CPU (the last frame of a robot camera), shown
/// in an ImGui panel.
struct ShownPicture
{
    compages::gpu::Texture color;
    std::uint32_t width = 0;
    std::uint32_t height = 0;

    bool resize(std::uint32_t p_width, std::uint32_t p_height);
};

// A robot camera rendered by Compages: the image source of a RobotIK camera.
class RenderedCamera final: public robotik::FrameSource
{
public:

    /// The camera does not see the helps of the operator (teach axes,
    /// waypoint markers): they are marked RenderFlags::Helper.
    RenderedCamera(compages::renderer::Scene& p_scene,
                   robotik::Camera const& p_camera,
                   compages::world::Entity p_link);

    bool capture(robotik::Camera const& p_camera,
                 robotik::CameraFrame& p_frame) override;
    void sync(robotik::Camera const& p_camera);

    [[nodiscard]] RenderTarget const& target() const
    {
        return m_target;
    }

private:

    compages::renderer::Scene& m_scene;
    compages::world::Entity m_entity;
    RenderTarget m_target;
};

// Meshes and cameras of the simulator: everything RobotIK does not render.
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
    void actor(compages::world::Entity p_entity,
               std::filesystem::path const& p_model,
               double p_yaw,
               double p_scale) override;
    void animate(compages::world::Entity p_entity,
                 std::string const& p_clip,
                 robotik::Seconds p_fade,
                 bool p_loop,
                 double p_ground_speed) override;
    void say(compages::world::Entity p_entity,
             std::string const& p_text,
             double p_height) override;
    void shown(compages::world::Entity p_entity, bool p_shown) override;

    /// Marks @p_entity as a help for the operator: the 3D view shows
    /// it, the robot cameras never see it (it would fool perception).
    void overlay(compages::world::Entity p_entity)
    {
        p_entity.get<compages::renderer::MeshRenderer>().flags =
            compages::renderer::RenderFlags::Helper;
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
    [[nodiscard]] ShownPicture const& frameTarget(std::size_t p_index);

private:

    compages::renderer::Scene& m_scene;
    std::vector<std::unique_ptr<RenderedCamera>> m_cameras;
    std::vector<std::unique_ptr<ShownPicture>> m_frames;
    bool m_grounded = false;
    std::filesystem::path m_ground_file;
    compages::renderer::CanvasTexture m_paint;
    compages::world::Entity m_sheet;
    double m_sheet_width = 1.0;
    double m_sheet_height = 1.0;
    bool m_sheet_ready = false;

    /// The model of an actor and what was measured on its clips.
    struct ActorModel
    {
        compages::world::Entity model;
        /// Entity of the speech bubble, above the head.
        compages::world::Entity bubble;
        /// Ground speed of each walk or run clip (m/s), measured once.
        std::map<std::string, double, std::less<>> strides;
        /// Clips the model does not have: asked once, not again.
        std::set<std::string, std::less<>> missing;
        compages::Vector3f scale{ 1.0F, 1.0F, 1.0F };
    };
    /// @return The model of an actor entity, or null.
    ActorModel* actorModel(compages::world::Entity p_entity);
    std::map<compages::world::EntityId, ActorModel> m_actors;
    bool m_font = false;
};
