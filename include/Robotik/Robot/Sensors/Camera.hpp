// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file Camera.hpp
/// Robot camera sensor, its frames and the source that fills them.
///
/// Robotik owns the camera concept; it does not render nor grab images. A
/// @ref FrameSource does: the simulator renders with Compages, a demo grabs a
/// USB camera, a test synthesizes pixels. Consumers only see @ref CameraFrame
/// (CPU images), never OpenGL, OpenCV or a driver.
///
/// Optical frame convention (OpenCV, AprilTag, ROS): @c +Z looks forward,
/// @c +X goes right in the image and @c +Y goes down.
#pragma once

#include "Robotik/Common/Math.hpp"
#include "Robotik/Common/Signal.hpp"
#include "Robotik/Common/Random.hpp"
#include "Robotik/Robot/Sensors/Image.hpp"
#include "Robotik/Robot/Sensors/Measurements.hpp"
#include "Robotik/Robot/Sensors/Sensor.hpp"

#include <array>
#include <functional>
#include <optional>
#include <string>
#include <vector>

namespace robotik
{

// ****************************************************************************
/// Pinhole model: image size, focal lengths and principal point (px).
///
/// @code
/// auto const k = robotik::CameraIntrinsics::fromFov(640, 480, Radians(1.0));
/// robotik::Vector3 const ray = k.ray(320.0, 240.0);       // straight ahead:
/// +Z if (auto const px = k.project(robotik::Vector3(0.1, 0.0, 1.0)))
///     std::cout << (*px)[0] << ", " << (*px)[1] << '\n';
/// @endcode
// ****************************************************************************
struct CameraIntrinsics
{
    std::uint32_t width = 320;  ///< Image width (px).
    std::uint32_t height = 240; ///< Image height (px).
    double fx = 171.0;          ///< Horizontal focal length (px).
    double fy = 171.0;          ///< Vertical focal length (px).
    double cx = 160.0;          ///< Principal point, column (px).
    double cy = 120.0;          ///< Principal point, row (px).

    // -------------------------------------------------------------------------
    /// Builds square pixels with a centered principal point.
    /// @param p_width Image width (px).
    /// @param p_height Image height (px).
    /// @param p_fov Vertical field of view.
    /// @return The intrinsics.
    // -------------------------------------------------------------------------
    [[nodiscard]] static CameraIntrinsics
    fromFov(std::uint32_t p_width, std::uint32_t p_height, Radians p_fov);

    /// @return The vertical field of view.
    [[nodiscard]] Radians fov() const;

    // -------------------------------------------------------------------------
    /// @param p_u Column (px).
    /// @param p_v Row (px).
    /// @return Unit ray through the pixel, in the optical frame.
    // -------------------------------------------------------------------------
    [[nodiscard]] Vector3 ray(double p_u, double p_v) const;

    // -------------------------------------------------------------------------
    /// @param p_point Point in the optical frame (m).
    /// @return Its pixel (column, row), or nothing when it is behind the
    ///   camera.
    // -------------------------------------------------------------------------
    [[nodiscard]] std::optional<std::array<double, 2>>
    project(Vector3 const& p_point) const;
};

// ****************************************************************************
/// One capture: images plus where and when they were taken.
///
/// @code
/// robotik::CameraFrame const& f = camera.frame();
/// if (f.valid && !f.depth.empty())
///     float const z = f.depth.row<float>(f.intrinsics.height /
///     2)[f.intrinsics.width / 2];
/// @endcode
// ****************************************************************************
struct CameraFrame: Measurement
{
    /// Color image (@ref PixelFormat::RGB8 or Gray8).
    Image rgb;
    /// Depth image (@ref PixelFormat::Depth32F), empty for an RGB camera.
    Image depth;
    /// Intrinsics the images were taken with.
    CameraIntrinsics intrinsics;
    /// Optical frame in the robot base frame at capture time.
    Pose pose;
};

class Camera;

// ****************************************************************************
/// Fills a @ref CameraFrame: renderer, driver or synthetic generator.
///
/// @code
/// class GrayTest final : public robotik::FrameSource
/// {
/// public:
///     bool capture(robotik::Camera const&, robotik::CameraFrame& p_frame)
///     override
///     {
///         auto const& k = p_frame.intrinsics;
///         p_frame.rgb.resize(k.width, k.height, robotik::PixelFormat::RGB8);
///         std::ranges::fill(p_frame.rgb.bytes(), std::uint8_t(128));
///         return true;
///     }
/// };
/// @endcode
// ****************************************************************************
class FrameSource
{
public:

    virtual ~FrameSource() = default;

    // -------------------------------------------------------------------------
    /// Writes the images of a frame.
    /// @param p_camera Camera asking for the images.
    /// @param p_frame Frame to fill; its pose and intrinsics are already set.
    /// @return False when no image could be produced.
    // -------------------------------------------------------------------------
    virtual bool capture(Camera const& p_camera, CameraFrame& p_frame) = 0;
};

// ****************************************************************************
/// Where the camera is and how it samples.
///
/// @code
/// robotik::CameraConfig config;
/// config.parent = "link6";
/// config.intrinsics = robotik::CameraIntrinsics::fromFov(320, 240,
/// Radians(0.9)); config.frequency = 15.0; config.depth = true;
/// @endcode
// ****************************************************************************
struct CameraConfig
{
    /// Link carrying the camera; empty for the robot base.
    std::string parent;
    /// Optical frame in the parent link frame.
    Pose mount;
    /// Image geometry.
    CameraIntrinsics intrinsics;
    /// Capture rate in Hz.
    double frequency = 30.0;
    /// True for an RGB-D camera (the source should fill the depth image).
    bool depth = false;
    /// Gaussian noise on color channels, as a fraction of 255 (0: none).
    double noise = 0.0;
};

// ****************************************************************************
/// A camera of the robot.
///
/// Frames are handed to the callbacks as soon as they are captured: a
/// camera has no latency model (it would have to queue images). Without a
/// source (headless run) it produces no frame.
///
/// @code
/// auto& camera = robot.sensors().add<robotik::Camera>(
///     "wrist_camera", robotik::CameraConfig{ .parent = "link6" });
/// camera.source(&renderer_camera);
/// camera.onFrame([&](robotik::CameraFrame const& p_frame) {
///     pipeline.process(p_frame);
/// });
/// @endcode
// ****************************************************************************
class Camera final: public Sensor
{
public:

    /// Function called with each new frame.
    using Callback = std::function<void(CameraFrame const&)>;

    // -------------------------------------------------------------------------
    /// @param p_name Camera name, also its resource name.
    /// @param p_config Mount, intrinsics, rate, depth and noise.
    // -------------------------------------------------------------------------
    Camera(std::string p_name, CameraConfig p_config = {});

    /// @return The configuration.
    [[nodiscard]] CameraConfig const& config() const
    {
        return m_config;
    }

    // -------------------------------------------------------------------------
    /// Moves the camera on its link (mission / calibration).
    /// @param p_mount Optical frame in the parent link.
    // -------------------------------------------------------------------------
    void mount(Pose const& p_mount)
    {
        m_config.mount = p_mount;
    }

    /// @return The image geometry.
    [[nodiscard]] CameraIntrinsics const& intrinsics() const
    {
        return m_config.intrinsics;
    }

    // -------------------------------------------------------------------------
    /// Connects the image source.
    /// @param p_source Source, not owned; null disconnects.
    // -------------------------------------------------------------------------
    void source(FrameSource* p_source)
    {
        m_source = p_source;
    }

    /// @return The image source, or null.
    [[nodiscard]] FrameSource* source() const
    {
        return m_source;
    }

    // -------------------------------------------------------------------------
    /// @return Last captured frame (empty images before the first capture).
    // -------------------------------------------------------------------------
    [[nodiscard]] CameraFrame const& frame() const
    {
        return m_frame;
    }

    /// Emitted after every capture with the new frame, to the slots in
    /// connection order.
    Signal<CameraFrame const&> frames;

    // -------------------------------------------------------------------------
    /// Calls a function after every capture (a shortcut for
    /// @c frames.connect).
    /// @param p_callback Function receiving the frame.
    /// @return Handle to disconnect it from @ref frames.
    // -------------------------------------------------------------------------
    Signal<CameraFrame const&>::Id onFrame(Callback p_callback)
    {
        return frames.connect(std::move(p_callback));
    }

    // -------------------------------------------------------------------------
    /// Reseeds the noise generator (episode reset).
    /// @param p_seed Same seed, same noise.
    // -------------------------------------------------------------------------
    void seed(Seed p_seed)
    {
        m_random = Random(p_seed);
    }

    /// Marks the current frame invalid (the sensor resource failed).
    void invalidate() override
    {
        m_frame.valid = false;
    }

protected:

    /// Asks the source for images, adds the noise, calls the callbacks.
    bool sample(Robot const& p_robot, Seconds p_now) override;

private:

    /// Mount, intrinsics, rate, depth and noise.
    CameraConfig m_config;
    /// Image source, not owned.
    FrameSource* m_source = nullptr;
    /// Last captured frame.
    CameraFrame m_frame;
    /// Noise generator.
    Random m_random;
};

} // namespace robotik
