// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "SimulatorView.hpp"

#include "SimulatorDisplay.hpp"

#include "Robotik/Robot/Components/Objects.hpp"
#include "Robotik/Robot/Sensors/Image.hpp"

#include "Compages/GPU/RenderPass.hpp"
#include "Compages/Renderer/Procedural/CanvasTexture.hpp"
#include "Compages/Renderer/Components/MeshRenderer.hpp"
#include "Compages/World/Components/Camera.hpp"

#include <algorithm>
#include <vector>

#include <cmath>
#include <cstring>
#include <fstream>
#include <iostream>
#include <numbers>
#include <stdexcept>

bool ShownPicture::resize(std::uint32_t p_width, std::uint32_t p_height)
{
    if (p_width == width && p_height == height)
    {
        return width != 0;
    }
    width = 0;
    height = 0;
    auto made = compages::gpu::Texture::create(
        { .format = compages::gpu::PixelFormat::RGB8, .width = p_width, .height = p_height });
    if (!made)
    {
        std::cerr << "Camera picture: " << made.error() << '\n';
        return false;
    }
    color = std::move(made.value());
    width = p_width;
    height = p_height;
    return true;
}

RenderedCamera::RenderedCamera(
    compages::renderer::Scene& p_scene,
    robotik::Camera const& p_camera,
    compages::world::Entity p_link)
    : m_scene(p_scene)
{
    compages::world::Entity const active = p_scene.activeCamera();

    m_entity = p_scene.camera(p_camera.name()).parent(p_link);
    sync(p_camera);
    auto& lens = m_entity.get<compages::world::Camera>();
    lens.fov = units::angle::radian_t(p_camera.intrinsics().fov().value());
    lens.near_plane = 0.03F;
    lens.far_plane = 20.0F;
    // The robot does not see the helps drawn for the operator.
    lens.sees_helpers = false;
    if (active)
    {
        p_scene.activeCamera(active);
    }
}

bool RenderedCamera::capture(robotik::Camera const& p_camera,
                             robotik::CameraFrame& p_frame)
{
    robotik::CameraIntrinsics const& intrinsics = p_camera.intrinsics();
    if (auto sized = m_target.resize(intrinsics.width, intrinsics.height); !sized)
    {
        std::cerr << "Robot camera: " << sized.error() << '\n';
        return false;
    }
    sync(p_camera);
    m_scene.render(m_entity, m_target);
    // RGBA, the top row first, as Robotik images: only the alpha goes.
    auto pixels = m_target.readColor();
    if (!pixels)
    {
        return false;
    }
    std::vector<std::byte> const& bytes = pixels.value();
    p_frame.rgb.resize(
        intrinsics.width, intrinsics.height, robotik::PixelFormat::RGB8);
    if (bytes.size() < std::size_t(intrinsics.width) * intrinsics.height * 4U)
    {
        return false;
    }
    for (std::uint32_t y = 0; y < intrinsics.height; ++y)
    {
        auto* row = p_frame.rgb.row<std::uint8_t>(y);
        std::byte const* from = bytes.data() + std::size_t(y) * intrinsics.width * 4U;
        for (std::uint32_t x = 0; x < intrinsics.width; ++x)
        {
            row[3U * x] = std::to_integer<std::uint8_t>(from[4U * x]);
            row[3U * x + 1U] = std::to_integer<std::uint8_t>(from[4U * x + 1U]);
            row[3U * x + 2U] = std::to_integer<std::uint8_t>(from[4U * x + 2U]);
        }
    }
    // An RGB-D camera: the distance along the optical axis, in metres
    // (+infinity where nothing was drawn).
    if (p_camera.config().depth)
    {
        auto metres = m_target.readDepth(m_entity.get<compages::world::Camera>());
        if (!metres)
        {
            return false;
        }
        p_frame.depth.resize(intrinsics.width, intrinsics.height, robotik::PixelFormat::Depth32F);
        for (std::uint32_t y = 0; y < intrinsics.height; ++y)
        {
            std::memcpy(p_frame.depth.row<float>(y),
                        metres.value().data() + std::size_t(y) * intrinsics.width,
                        std::size_t(intrinsics.width) * sizeof(float));
        }
    }
    else
    {
        p_frame.depth = robotik::Image{};
    }
    return true;
}

void RenderedCamera::sync(robotik::Camera const& p_camera)
{
    // Compages cameras look down their -Z, optical frames down their +Z.
    robotik::Pose const& mount = p_camera.config().mount;
    robotik::Quaternion const rotation =
        (mount.rotation *
         robotik::Quaternion::fromAngleAxis(robotik::Radians(std::numbers::pi), { 1.0, 0.0, 0.0 }))
            .normalized();
    m_entity
        .position(static_cast<float>(mount.position.x),
                  static_cast<float>(mount.position.y),
                  static_cast<float>(mount.position.z))
        .rotation(compages::Quatf(static_cast<float>(rotation.w),
                                        static_cast<float>(rotation.x),
                                        static_cast<float>(rotation.y),
                                        static_cast<float>(rotation.z)));
}

compages::world::Entity
SimulatorView::model(compages::world::World& /*p_world*/,
                     std::filesystem::path const& p_urdf)
{
    auto loaded = m_scene.loadFile(p_urdf);
    if (!loaded)
    {
        throw std::runtime_error(loaded.error());
    }
    return loaded.value();
}

compages::world::Entity
SimulatorView::robot(compages::world::World& /*p_world*/,
                     std::filesystem::path const& p_urdf)
{
    auto loaded = m_scene.loadFile(p_urdf);
    if (!loaded)
    {
        throw std::runtime_error(loaded.error());
    }
    return loaded.value();
}

void SimulatorView::object(compages::world::Entity p_entity,
                           robotik::ecs::Shape const& p_object)
{
    auto const look = compages::renderer::color(
        p_object.color[0], p_object.color[1], p_object.color[2]);
    auto const x = static_cast<float>(p_object.size[0].value());
    auto const y = static_cast<float>(p_object.size[1].value());
    auto const z = static_cast<float>(p_object.size[2].value());
    auto part = [&](char const* p_name,
                    float p_px,
                    float p_py,
                    float p_pz,
                    float p_sx,
                    float p_sy,
                    float p_sz)
    {
        m_scene.box(p_object.name + p_name, look)
            .parent(p_entity)
            .position(p_px, p_py, p_pz)
            .scale(p_sx, p_sy, p_sz);
    };
    if (p_object.type == robotik::ecs::Shape::Type::BOX)
    {
        part("_mesh", 0.0F, 0.0F, 0.0F, x, y, z);
        return;
    }
    // Open-top bin: same wall thickness (m) as scene::innerBounds /
    // CONTAINER_WALL_M. The parts do not overlap: two faces in the same plane
    // would flicker. North and south run the full length, east and west fit
    // between them, the floor fits inside the four walls.
    float const wall = static_cast<float>(robotik::ecs::CONTAINER_WALL_M);
    float const inner_x = std::max(x - 2.0F * wall, 0.0F);
    float const inner_y = std::max(y - 2.0F * wall, 0.0F);
    part("_floor", 0.0F, 0.0F, (wall - z) * 0.5F, inner_x, inner_y, wall);
    part("_north", 0.0F, (y - wall) * 0.5F, 0.0F, x, wall, z);
    part("_south", 0.0F, (wall - y) * 0.5F, 0.0F, x, wall, z);
    part("_east", (x - wall) * 0.5F, 0.0F, 0.0F, wall, inner_y, z);
    part("_west", (wall - x) * 0.5F, 0.0F, 0.0F, wall, inner_y, z);
}

void noShadow(compages::world::Entity p_entity)
{
    if (auto* mesh = p_entity.find<compages::renderer::MeshRenderer>())
    {
        mesh->flags =
            mesh->flags | compages::renderer::RenderFlags::NoShadowCaster;
    }
}

std::filesystem::path floorGrid()
{
    // 3 m across: a line every 10 cm, a stronger one every 50 cm.
    constexpr int SIZE = 1500;
    constexpr int CELL = SIZE / 30;
    std::filesystem::path const path =
        std::filesystem::temp_directory_path() / "robotik_floor.ppm";
    std::ofstream file(path, std::ios::binary);
    file << "P6\n" << SIZE << ' ' << SIZE << "\n255\n";
    std::vector<unsigned char> row(static_cast<std::size_t>(SIZE) * 3U);
    for (int v = 0; v < SIZE; ++v)
    {
        for (int u = 0; u < SIZE; ++u)
        {
            auto line = [](int p_at, int p_every, int p_width)
            { return p_at % p_every < p_width; };
            bool const major = line(u, CELL * 5, 4) || line(v, CELL * 5, 4);
            bool const minor = line(u, CELL, 2) || line(v, CELL, 2);
            unsigned char const shade = major ? 58 : minor ? 70 : 82;
            std::size_t const i = static_cast<std::size_t>(u) * 3U;
            row[i] = shade;
            row[i + 1U] = static_cast<unsigned char>(shade + 2);
            row[i + 2U] = static_cast<unsigned char>(shade + 5);
        }
        file.write(reinterpret_cast<char const*>(row.data()),
                   static_cast<std::streamsize>(row.size()));
    }
    return path;
}

robotik::FrameSource* SimulatorView::camera(robotik::Camera& p_camera,
                                            compages::world::Entity p_link)
{
    m_cameras.push_back(std::make_unique<RenderedCamera>(
        m_scene, p_camera, p_link));
    return m_cameras.back().get();
}

void SimulatorView::ground(robotik::Image const& p_image,
                           double p_width,
                           double p_height)
{
    if (p_image.format() != robotik::PixelFormat::RGB8 || p_image.empty())
    {
        return;
    }
    m_ground_file =
        std::filesystem::temp_directory_path() / "robotik_ground.ppm";
    std::ofstream file(m_ground_file, std::ios::binary);
    file << "P6\n" << p_image.width() << ' ' << p_image.height() << "\n255\n";
    file.write(reinterpret_cast<char const*>(p_image.data()),
               static_cast<std::streamsize>(p_image.bytes().size()));
    if (!file)
    {
        return;
    }
    file.close();
    // Plane is XY facing +Z; Compages world is Y-up, URDF ground is Z-up.
    compages::world::Entity ground = m_scene.plane(
        "Ground", compages::renderer::texture(m_ground_file.string()));
    ground
        .rotation(robotik::Radians(-0.5F * std::numbers::pi_v<float>),
                  compages::Vector3f(1.0F, 0.0F, 0.0F))
        .position(0.0F, 0.0F, 0.0F)
        .scale(static_cast<float>(p_width), static_cast<float>(p_height), 1.0F);
    noShadow(ground);
    m_grounded = true;
}

bool SimulatorView::openCanvas(compages::world::Entity p_parent,
                               double p_width,
                               double p_height)
{
    if (!p_parent || p_width < 1e-4 || p_height < 1e-4)
    {
        return false;
    }
    if (!m_paint.create(512U, 512U))
    {
        return false;
    }
    if (!m_paint.bind(m_scene.assets(), "painter_canvas"))
    {
        return false;
    }
    // Unit plane faces +Z and lies in XY. This quaternion sends that +Z onto
    // the parent's +X (the painted normal) and the plane's +Y onto +Z.
    m_sheet = m_scene.plane("CanvasPaint")
                  .parent(p_parent)
                  .position(0.012F, 0.0F, 0.0F)
                  .rotation(compages::Quatf(0.5F, 0.5F, 0.5F, 0.5F))
                  .scale(static_cast<float>(p_width),
                         static_cast<float>(p_height),
                         1.0F);
    m_paint.applyLook(m_scene, m_sheet.id());
    m_sheet_width = p_width;
    m_sheet_height = p_height;
    m_sheet_ready = true;
    return true;
}

void SimulatorView::ink(double p_y,
                        double p_z,
                        float p_red,
                        float p_green,
                        float p_blue)
{
    if (!m_sheet_ready)
    {
        return;
    }
    auto const color = compages::renderer::CanvasTexture::Color::rgb(
        static_cast<std::uint8_t>(p_red * 255.0F),
        static_cast<std::uint8_t>(p_green * 255.0F),
        static_cast<std::uint8_t>(p_blue * 255.0F));
    // Mapping covers the unit plane (-0.5..0.5). Scale turns that into metres.
    float const u = static_cast<float>(p_y / m_sheet_width);
    float const v = static_cast<float>(p_z / m_sheet_height);
    m_paint.paintLocal({ u, v, 0.0F }, color, 0.035F);
    (void)m_paint.sync();
}

ShownPicture const& SimulatorView::frameTarget(std::size_t p_index)
{
    while (m_frames.size() <= p_index)
    {
        m_frames.push_back(std::make_unique<ShownPicture>());
    }
    return *m_frames[p_index];
}

bool SimulatorView::showFrame(std::size_t p_index,
                              robotik::CameraFrame const& p_frame)
{
    robotik::Image const& image = p_frame.rgb;
    if (image.format() != robotik::PixelFormat::RGB8 || image.empty())
    {
        return false;
    }
    (void)frameTarget(p_index);
    ShownPicture& target = *m_frames[p_index];
    if (!target.resize(image.width(), image.height()))
    {
        return false;
    }
    std::vector<std::byte> pixels(image.bytes().size());
    std::size_t const stride = image.stride();
    for (std::uint32_t y = 0; y < image.height(); ++y)
    {
        std::memcpy(pixels.data() + (image.height() - 1U - y) * stride,
                    image.row<std::uint8_t>(y),
                    stride);
    }
    return static_cast<bool>(target.color.write(pixels));
}
