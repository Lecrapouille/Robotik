// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "Floor.hpp"

extern "C" {
#include "apriltag.h"
#include "tag36h11.h"
}

#include <algorithm>
#include <cmath>
#include <numbers>

#define FLOOR_MARGIN_M 0.6
#define FLOOR_GREY 196
#define LINE_GREY 24
#define SKY_R 150
#define SKY_G 170
#define SKY_B 200

FloorMap::FloorMap(Track const& p_track,
                   std::vector<FloorTag> p_tags,
                   double p_tag_size,
                   double p_resolution)
    : m_tag_size(p_tag_size), m_resolution(p_resolution)
{
    double const half_x = 0.5 * p_track.straight + p_track.radius + FLOOR_MARGIN_M;
    double const half_y = p_track.radius + FLOOR_MARGIN_M;
    m_x0 = -half_x;
    m_y1 = half_y;
    auto const width = static_cast<std::uint32_t>(std::ceil(2.0 * half_x / p_resolution));
    auto const height = static_cast<std::uint32_t>(std::ceil(2.0 * half_y / p_resolution));
    m_image.resize(width, height, robotik::PixelFormat::RGB8);

    for (std::uint32_t row = 0; row < height; ++row)
    {
        std::uint8_t* pixel = m_image.row<std::uint8_t>(row);
        double const y = m_y1 - (row + 0.5) * p_resolution;
        for (std::uint32_t col = 0; col < width; ++col, pixel += 3)
        {
            double const x = m_x0 + (col + 0.5) * p_resolution;
            Track::Point const near = p_track.closest(x, y);
            bool const line = std::hypot(x - near.x, y - near.y) <= 0.5 * p_track.width;
            std::uint8_t const grey = line ? LINE_GREY : FLOOR_GREY;
            pixel[0] = grey;
            pixel[1] = grey;
            pixel[2] = grey;
        }
    }

    // Tag image: one byte per module, white outer border included. Its
    // columns go along the map X axis and its rows along -Y.
    apriltag_family_t* family = tag36h11_create();
    double const module = p_tag_size / family->width_at_border;
    double const total = family->total_width;
    for (FloorTag const& tag : p_tags)
    {
        image_u8_t* picture = apriltag_to_image(family, static_cast<uint32_t>(tag.id));
        auto const [c0, r0] = pixel(tag.x - 0.5 * total * module, tag.y + 0.5 * total * module);
        auto const [c1, r1] = pixel(tag.x + 0.5 * total * module, tag.y - 0.5 * total * module);
        for (auto row = static_cast<std::uint32_t>(std::max(r0, 0.0));
             row < std::min<std::uint32_t>(height, static_cast<std::uint32_t>(r1 + 1.0)); ++row)
        {
            for (auto col = static_cast<std::uint32_t>(std::max(c0, 0.0));
                 col < std::min<std::uint32_t>(width, static_cast<std::uint32_t>(c1 + 1.0)); ++col)
            {
                double const x = m_x0 + (col + 0.5) * p_resolution;
                double const y = m_y1 - (row + 0.5) * p_resolution;
                int const u = static_cast<int>(std::floor((x - tag.x) / module + 0.5 * total));
                int const v = static_cast<int>(std::floor((tag.y - y) / module + 0.5 * total));
                if (u < 0 || v < 0 || u >= picture->width || v >= picture->height)
                {
                    continue;
                }
                std::uint8_t const value = picture->buf[v * picture->stride + u];
                std::uint8_t* out = m_image.row<std::uint8_t>(row) + 3 * col;
                out[0] = value;
                out[1] = value;
                out[2] = value;
            }
        }
        image_u8_destroy(picture);

        // Tag frame: X along the image columns, Y along its rows, Z into the
        // tag, i.e. down into the floor.
        m_landmarks.push_back(
            { tag.id,
              robotik::Pose{ { tag.x, tag.y, 0.0 },
                             robotik::Quaternion::axisAngle({ 1.0, 0.0, 0.0 }, std::numbers::pi) } });
    }
    tag36h11_destroy(family);
}

void FloorMap::sample(double p_x, double p_y, std::uint8_t* p_rgb) const
{
    auto const [u, v] = pixel(p_x, p_y);
    double const fu = u - 0.5;
    double const fv = v - 0.5;
    auto const width = static_cast<int>(m_image.width());
    auto const height = static_cast<int>(m_image.height());
    int const u0 = static_cast<int>(std::floor(fu));
    int const v0 = static_cast<int>(std::floor(fv));
    if (u0 < 0 || v0 < 0 || u0 + 1 >= width || v0 + 1 >= height)
    {
        p_rgb[0] = FLOOR_GREY;
        p_rgb[1] = FLOOR_GREY;
        p_rgb[2] = FLOOR_GREY;
        return;
    }
    double const a = fu - u0;
    double const b = fv - v0;
    std::uint8_t const* top = m_image.row<std::uint8_t>(static_cast<std::uint32_t>(v0)) + 3 * u0;
    std::uint8_t const* bottom = m_image.row<std::uint8_t>(static_cast<std::uint32_t>(v0 + 1)) + 3 * u0;
    for (int c = 0; c < 3; ++c)
    {
        double const value = (1.0 - b) * ((1.0 - a) * top[c] + a * top[3 + c]) +
                             b * ((1.0 - a) * bottom[c] + a * bottom[3 + c]);
        p_rgb[c] = static_cast<std::uint8_t>(value + 0.5);
    }
}

bool FloorCamera::capture(robotik::Camera const& p_camera, robotik::CameraFrame& p_frame)
{
    robotik::CameraIntrinsics const& intrinsics = p_camera.intrinsics();
    if (m_width != intrinsics.width || m_height != intrinsics.height)
    {
        m_width = intrinsics.width;
        m_height = intrinsics.height;
        m_rays.resize(std::size_t(m_width) * m_height);
        for (std::uint32_t v = 0; v < m_height; ++v)
        {
            for (std::uint32_t u = 0; u < m_width; ++u)
            {
                m_rays[v * m_width + u] = intrinsics.ray(u, v);
            }
        }
    }

    robotik::Pose const optical = m_drive.pose() * p_frame.pose;
    p_frame.rgb.resize(m_width, m_height, robotik::PixelFormat::RGB8);
    for (std::uint32_t v = 0; v < m_height; ++v)
    {
        std::uint8_t* pixel = p_frame.rgb.row<std::uint8_t>(v);
        robotik::Vector3 const* ray = m_rays.data() + v * m_width;
        for (std::uint32_t u = 0; u < m_width; ++u, pixel += 3)
        {
            robotik::Vector3 const direction = optical.rotation.rotate(ray[u]);
            if (direction.z > -1e-6)
            {
                pixel[0] = SKY_R;
                pixel[1] = SKY_G;
                pixel[2] = SKY_B;
                continue;
            }
            double const t = -optical.position.z / direction.z;
            m_floor.sample(optical.position.x + t * direction.x,
                           optical.position.y + t * direction.y,
                           pixel);
        }
    }
    return true;
}
