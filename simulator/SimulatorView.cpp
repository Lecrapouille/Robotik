// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "SimulatorView.hpp"

#include "SimulatorDisplay.hpp"

#include "Robotik/ECS/ObjectComponents.hpp"
#include "Robotik/Sensors/Image.hpp"

#include "Compages/GPU/RenderPass.hpp"
#include "Compages/World/Components/Camera.hpp"

#include <cmath>
#include <cstring>
#include <fstream>
#include <iostream>
#include <numbers>
#include <stdexcept>

bool RenderTarget::resize(std::uint32_t p_width, std::uint32_t p_height)
{
    if (p_width == width && p_height == height)
    {
        return width != 0;
    }
    width = 0;
    height = 0;
    compages::Status made =
        color.allocate({ .format = compages::gpu::PixelFormat::RGB8,
                         .width = p_width,
                         .height = p_height });
    if (made)
    {
        made = depth.allocate({ .format = compages::gpu::PixelFormat::Depth32F,
                                .width = p_width,
                                .height = p_height,
                                .magnify = compages::gpu::Filter::Nearest,
                                .minify = compages::gpu::Filter::Nearest });
    }
    if (made)
    {
        made = framebuffer.attach(color, depth);
    }
    if (!made)
    {
        std::cerr << "Render target: " << made.error() << '\n';
        return false;
    }
    width = p_width;
    height = p_height;
    return true;
}

RenderedCamera::RenderedCamera(compages::renderer::Scene& p_scene,
                               robotik::Camera const& p_camera,
                               compages::world::Entity p_link)
    : m_scene(p_scene)
{
    compages::world::Entity const active = p_scene.activeCamera();

    m_entity = p_scene.camera(p_camera.name()).parent(p_link);
    sync(p_camera);
    auto& lens = m_entity.get<compages::world::Camera>();
    lens.fov = units::angle::radian_t(p_camera.intrinsics().fov().value());
    lens.near_plane = 0.01f;
    lens.far_plane = 20.0f;
    if (active)
    {
        p_scene.activeCamera(active);
    }
}

bool RenderedCamera::capture(robotik::Camera const& p_camera,
                             robotik::CameraFrame& p_frame)
{
    robotik::CameraIntrinsics const& intrinsics = p_camera.intrinsics();
    if (!m_target.resize(intrinsics.width, intrinsics.height))
    {
        return false;
    }
    sync(p_camera);
    {
        compages::gpu::RenderPass pass(
            m_target.framebuffer,
            compages::gpu::PassDesc{ .color = viewClearColor() });
        m_scene.render(m_entity);
    }
    auto pixels = m_target.color.read();
    if (!pixels)
    {
        return false;
    }

    // The device puts the first row at the bottom; Robotik images start at
    // the top.
    std::vector<std::byte> const& bytes = pixels.value();
    p_frame.rgb.resize(intrinsics.width, intrinsics.height, robotik::PixelFormat::RGB8);
    std::size_t const stride = p_frame.rgb.stride();
    if (bytes.size() < stride * intrinsics.height)
    {
        return false;
    }
    for (std::uint32_t y = 0; y < intrinsics.height; ++y)
    {
        std::memcpy(p_frame.rgb.row<std::uint8_t>(y),
                    bytes.data() + (intrinsics.height - 1u - y) * stride,
                    stride);
    }
    return true;
}

void RenderedCamera::sync(robotik::Camera const& p_camera)
{
    // Compages cameras look down their -Z, optical frames down their +Z.
    robotik::Pose const& mount = p_camera.config().mount;
    robotik::Quaternion const rotation =
        (mount.rotation *
         robotik::axisAngle({ 1.0, 0.0, 0.0 }, std::numbers::pi))
            .normalized();
    m_entity
        .position(static_cast<float>(mount.position.x),
                  static_cast<float>(mount.position.y),
                  static_cast<float>(mount.position.z))
        .rotation(compages::core::Quatf(static_cast<float>(rotation.w),
                                        static_cast<float>(rotation.x),
                                        static_cast<float>(rotation.y),
                                        static_cast<float>(rotation.z)));
}

compages::world::Entity SimulatorView::robot(compages::world::World& /*p_world*/,
                                             std::filesystem::path const& p_urdf)
{
    auto loaded = m_scene.load(p_urdf.string());
    if (!loaded)
    {
        throw std::runtime_error(loaded.error());
    }
    return loaded.value();
}

void SimulatorView::object(compages::world::Entity p_entity,
                           robotik::ecs::SceneObject const& p_object)
{
    auto const look = compages::renderer::color(
        p_object.color[0], p_object.color[1], p_object.color[2]);
    auto const x = static_cast<float>(p_object.size[0].value());
    auto const y = static_cast<float>(p_object.size[1].value());
    auto const z = static_cast<float>(p_object.size[2].value());
    auto part = [&](char const* p_name, float p_px, float p_py, float p_pz,
                    float p_sx, float p_sy, float p_sz)
    {
        m_scene.box(p_object.name + p_name, look)
            .parent(p_entity)
            .position(p_px, p_py, p_pz)
            .scale(p_sx, p_sy, p_sz);
    };
    if (p_object.type == robotik::ecs::SceneObject::Type::CUBE)
    {
        part("_mesh", 0.0f, 0.0f, 0.0f, x, y, z);
        return;
    }
    // Open-top bin: same wall thickness (m) as scene::innerBounds / CONTAINER_WALL_M.
    float const wall = static_cast<float>(robotik::ecs::CONTAINER_WALL_M);
    part("_floor", 0.0f, 0.0f, (wall - z) * 0.5f, x, y, wall);
    part("_north", 0.0f, (y - wall) * 0.5f, 0.0f, x, wall, z);
    part("_south", 0.0f, (wall - y) * 0.5f, 0.0f, x, wall, z);
    part("_east", (x - wall) * 0.5f, 0.0f, 0.0f, wall, y, z);
    part("_west", (wall - x) * 0.5f, 0.0f, 0.0f, wall, y, z);
}

robotik::FrameSource* SimulatorView::camera(robotik::Camera& p_camera,
                                            compages::world::Entity p_link)
{
    m_cameras.push_back(std::make_unique<RenderedCamera>(m_scene, p_camera, p_link));
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
    m_ground_file = std::filesystem::temp_directory_path() / "robotik_ground.ppm";
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
    m_scene.plane("Ground", compages::renderer::texture(m_ground_file.string()))
        .rotation(Radians(-0.5f * std::numbers::pi_v<float>),
                  compages::core::Vector3f(1.0f, 0.0f, 0.0f))
        .position(0.0f, 0.0f, 0.0f)
        .scale(static_cast<float>(p_width), static_cast<float>(p_height), 1.0f);
    m_grounded = true;
}

bool SimulatorView::showFrame(robotik::CameraFrame const& p_frame)
{
    robotik::Image const& image = p_frame.rgb;
    if (image.format() != robotik::PixelFormat::RGB8 || image.empty())
    {
        return false;
    }
    if (!m_frame.resize(image.width(), image.height()))
    {
        return false;
    }
    std::vector<std::byte> pixels(image.bytes().size());
    std::size_t const stride = image.stride();
    for (std::uint32_t y = 0; y < image.height(); ++y)
    {
        std::memcpy(pixels.data() + (image.height() - 1u - y) * stride,
                    image.row<std::uint8_t>(y),
                    stride);
    }
    return static_cast<bool>(m_frame.color.write(pixels));
}
