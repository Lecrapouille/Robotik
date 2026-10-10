// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#pragma once

#include "DiffDrive.hpp"
#include "Track.hpp"

#include "Robotik/Perception/Localization.hpp"
#include "Robotik/Sensors/Camera.hpp"

#include <array>
#include <cstdint>
#include <vector>

// AprilTag (tag36h11) printed flat on the floor, aligned with the map axes.
struct FloorTag
{
    int id = 0;
    double x = 0.0;
    double y = 0.0;
};

// Top-down picture of the floor: the track line and the tags, the map the
// camera sees.
class FloorMap
{
public:

    // @p_tag_size is the edge of the black square (m), as the pose estimation
    // expects it.
    FloorMap(Track const& p_track,
             std::vector<FloorTag> p_tags,
             double p_tag_size,
             double p_resolution = 0.004);

    // Color under a map point (bilinear).
    void sample(double p_x, double p_y, std::uint8_t* p_rgb) const;

    // Pixel coordinates of a map point in @ref image.
    [[nodiscard]] std::array<double, 2> pixel(double p_x, double p_y) const
    {
        return { (p_x - m_x0) / m_resolution, (m_y1 - p_y) / m_resolution };
    }

    [[nodiscard]] robotik::Image const& image() const
    {
        return m_image;
    }

    // Tag frames in the map, in the convention of the AprilTag pose estimate.
    [[nodiscard]] std::vector<robotik::Landmark> const& landmarks() const
    {
        return m_landmarks;
    }

    [[nodiscard]] double tagSize() const
    {
        return m_tag_size;
    }

private:

    double m_tag_size;
    double m_resolution;
    double m_x0;
    double m_y1;
    robotik::Image m_image;
    std::vector<robotik::Landmark> m_landmarks;
};

// Simulated camera: casts each pixel ray onto the floor map. Plays the role
// of the renderer without a GPU, so the demo runs anywhere.
class FloorCamera final: public robotik::FrameSource
{
public:

    FloorCamera(FloorMap const& p_floor, DiffDriveBackend const& p_drive)
        : m_floor(p_floor), m_drive(p_drive)
    {
    }

    bool capture(robotik::Camera const& p_camera, robotik::CameraFrame& p_frame) override;

private:

    FloorMap const& m_floor;
    DiffDriveBackend const& m_drive;
    // Pixel rays in the optical frame, computed once per intrinsics.
    std::vector<robotik::Vector3> m_rays;
    std::uint32_t m_width = 0;
    std::uint32_t m_height = 0;
};
