// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

//! @file Camera.hpp
//! @brief Robot camera sensor, its frames and the source that fills them.
//!
//! Robotik owns the camera concept; it does not render nor grab images. A
//! @ref FrameSource does: the simulator renders with Compages, a demo grabs a
//! USB camera, a test synthesizes pixels. Consumers only see @ref CameraFrame
//! (CPU images), never OpenGL, OpenCV or a driver.
//!
//! Optical frame convention (OpenCV, AprilTag, ROS): @c +Z looks forward,
//! @c +X goes right in the image and @c +Y goes down.
#pragma once

#include "Robotik/Math/Pose.hpp"
#include "Robotik/Math/Random.hpp"
#include "Robotik/Sensors/Image.hpp"
#include "Robotik/Sensors/Sensor.hpp"

#include <array>
#include <functional>
#include <optional>
#include <string>
#include <vector>

namespace robotik
{

// ****************************************************************************
//! @brief Pinhole model: image size, focal lengths and principal point (px).
// ****************************************************************************
struct CameraIntrinsics
{
    std::uint32_t width = 320;
    std::uint32_t height = 240;
    double fx = 171.0;
    double fy = 171.0;
    double cx = 160.0;
    double cy = 120.0;

    // -------------------------------------------------------------------------
    //! @brief Square pixels, centered principal point, vertical field of view.
    // -------------------------------------------------------------------------
    [[nodiscard]] static CameraIntrinsics
    fromFov(std::uint32_t p_width, std::uint32_t p_height, Radians p_fov);

    //! @brief Vertical field of view.
    [[nodiscard]] Radians fov() const;

    // -------------------------------------------------------------------------
    //! @brief Unit ray through pixel (@p_u, @p_v), in the optical frame.
    // -------------------------------------------------------------------------
    [[nodiscard]] Vector3 ray(double p_u, double p_v) const;

    // -------------------------------------------------------------------------
    //! @brief Pixel of a point given in the optical frame, if in front.
    // -------------------------------------------------------------------------
    [[nodiscard]] std::optional<std::array<double, 2>>
    project(Vector3 const& p_point) const;
};

// ****************************************************************************
//! @brief One capture: images plus where and when they were taken.
// ****************************************************************************
struct CameraFrame
{
    //!< Color image (@ref PixelFormat::RGB8 or Gray8).
    Image rgb;
    //!< Depth image (@ref PixelFormat::Depth32F), empty for an RGB camera.
    Image depth;
    //!< Intrinsics the images were taken with.
    CameraIntrinsics intrinsics;
    //!< Optical frame in the robot base frame at capture time.
    Pose pose;
    //!< Capture time on the robot clock.
    Seconds stamp{};
    //!< Monotonic capture counter.
    std::uint64_t sequence = 0;
};

class Camera;

// ****************************************************************************
//! @brief Fills a @ref CameraFrame: renderer, driver or synthetic generator.
// ****************************************************************************
class FrameSource
{
public:

    virtual ~FrameSource() = default;

    // -------------------------------------------------------------------------
    //! @brief Writes the images of @p_frame (pose and intrinsics are set).
    //! @return False when no image could be produced.
    // -------------------------------------------------------------------------
    virtual bool capture(Camera const& p_camera, CameraFrame& p_frame) = 0;
};

// ****************************************************************************
//! @brief Where the camera is and how it samples.
// ****************************************************************************
struct CameraConfig
{
    //!< Link carrying the camera; empty for the robot base.
    std::string parent;
    //!< Optical frame in the parent link frame.
    Pose mount;
    //!< Image geometry.
    CameraIntrinsics intrinsics;
    //!< Capture rate in Hz.
    double frequency = 30.0;
    //!< True for an RGB-D camera (the source should fill the depth image).
    bool depth = false;
    //!< Gaussian noise on color channels, as a fraction of 255 (0: none).
    double noise = 0.0;
};

// ****************************************************************************
//! @brief A camera of the robot.
//!
//! @code
//! auto& camera = robot.sensors().add<robotik::Camera>(
//!     "wrist_camera", robotik::CameraConfig{ .parent = "link6" });
//! camera.source(&renderer_camera);
//! camera.onFrame([&](robotik::CameraFrame const& p_frame) {
//!     pipeline.process(p_frame);
//! });
//! @endcode
// ****************************************************************************
class Camera final: public Sensor
{
public:

    using Callback = std::function<void(CameraFrame const&)>;

    Camera(std::string p_name, CameraConfig p_config = {});

    [[nodiscard]] CameraConfig const& config() const
    {
        return m_config;
    }

    [[nodiscard]] CameraIntrinsics const& intrinsics() const
    {
        return m_config.intrinsics;
    }

    // -------------------------------------------------------------------------
    //! @brief Connects the image source (not owned); null disconnects.
    // -------------------------------------------------------------------------
    void source(FrameSource* p_source)
    {
        m_source = p_source;
    }

    [[nodiscard]] FrameSource* source() const
    {
        return m_source;
    }

    // -------------------------------------------------------------------------
    //! @brief Last captured frame (empty images before the first capture).
    // -------------------------------------------------------------------------
    [[nodiscard]] CameraFrame const& frame() const
    {
        return m_frame;
    }

    // -------------------------------------------------------------------------
    //! @brief Calls @p_callback after every capture, in registration order.
    // -------------------------------------------------------------------------
    void onFrame(Callback p_callback)
    {
        m_callbacks.push_back(std::move(p_callback));
    }

    // -------------------------------------------------------------------------
    //! @brief Reseeds the noise generator (episode reset).
    // -------------------------------------------------------------------------
    void seed(Seed p_seed)
    {
        m_random = Random(p_seed);
    }

protected:

    bool sample(Robot const& p_robot, Seconds p_now) override;

private:

    CameraConfig m_config;
    FrameSource* m_source = nullptr;
    CameraFrame m_frame;
    std::vector<Callback> m_callbacks;
    Random m_random;
};

} // namespace robotik
