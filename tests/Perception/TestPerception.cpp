// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "main.hpp"

#include "Robotik/Math/Random.hpp"
#include "Robotik/Perception/Detector.hpp"
#include "Robotik/Perception/Localization.hpp"
#include "Robotik/Perception/WorldModel.hpp"
#include "Robotik/Robot/Robot.hpp"
#include "Robotik/Sensors/Camera.hpp"

#include "Compages/World/World.hpp"

#include <numbers>

namespace
{

//! @brief Camera 1 m above the base origin, looking down.
robotik::Pose downward()
{
    return { { 0.0, 0.0, 1.0 },
             robotik::axisAngle({ 1.0, 0.0, 0.0 }, std::numbers::pi) };
}

//! @brief Gray image of the right size, counts captures.
class FlatSource final: public robotik::FrameSource
{
public:

    bool capture(robotik::Camera const& p_camera, robotik::CameraFrame& p_frame) override
    {
        auto const& intrinsics = p_camera.intrinsics();
        p_frame.rgb.resize(intrinsics.width, intrinsics.height, robotik::PixelFormat::RGB8);
        for (std::uint8_t& byte : p_frame.rgb.bytes())
        {
            byte = 128u;
        }
        ++captures;
        return true;
    }

    int captures = 0;
};

//! @brief Reports one object at a fixed pixel.
class FixedDetector final: public robotik::Detector
{
public:

    explicit FixedDetector(std::array<float, 2> p_center) : m_center(p_center) {}

    void detect(robotik::CameraFrame const&, robotik::Detections& p_detections) override
    {
        robotik::Detection detection;
        detection.label = "cube";
        detection.confidence = 0.9f;
        detection.center = m_center;
        p_detections.items.push_back(detection);
    }

private:

    std::array<float, 2> m_center;
};

} // namespace

TEST(Random, SeedsAreReplayableAndIndependent)
{
    robotik::Seed const master{ 42 };
    EXPECT_EQ(master.derive("world"), master.derive("world"));
    EXPECT_NE(master.derive("world").value, master.derive("faults").value);
    EXPECT_NE(master.derive(0u).value, master.derive(1u).value);

    robotik::Random a(master.derive("world"));
    robotik::Random b(master.derive("world"));
    for (int i = 0; i < 100; ++i)
    {
        double const x = a.uniform(-1.0, 1.0);
        EXPECT_EQ(x, b.uniform(-1.0, 1.0));
        EXPECT_GE(x, -1.0);
        EXPECT_LT(x, 1.0);
    }
}

TEST(Pose, ComposeAndInvert)
{
    robotik::Pose const pose{ { 1.0, 2.0, 3.0 }, robotik::rpy(0.1, 0.2, 0.3) };
    robotik::Pose const identity = pose * pose.inverse();
    EXPECT_NEAR(robotik::norm(identity.position), 0.0, 1e-12);
    EXPECT_NEAR(std::abs(identity.rotation.w), 1.0, 1e-12);
    robotik::Vector3 const point{ 0.5, -0.2, 0.1 };
    robotik::Vector3 const back = pose.inverse() * (pose * point);
    EXPECT_NEAR(robotik::norm(back - point), 0.0, 1e-12);
    EXPECT_NEAR(robotik::yawOf(robotik::rpy(0.0, 0.0, 0.7)), 0.7, 1e-12);
}

TEST(CameraIntrinsics, RayAndProjectAreInverse)
{
    auto const intrinsics = robotik::CameraIntrinsics::fromFov(
        320u, 240u, Radians(70.0 * std::numbers::pi / 180.0));
    EXPECT_NEAR(intrinsics.fov().value(), 70.0 * std::numbers::pi / 180.0, 1e-12);
    robotik::Vector3 const ray = intrinsics.ray(40.0, 200.0);
    auto const pixel = intrinsics.project(ray * 2.0);
    ASSERT_TRUE(pixel);
    EXPECT_NEAR((*pixel)[0], 40.0, 1e-9);
    EXPECT_NEAR((*pixel)[1], 200.0, 1e-9);
    EXPECT_FALSE(intrinsics.project({ 0.0, 0.0, -1.0 }));
}

TEST(Camera, SamplesAtItsFrequencyFromItsSource)
{
    compages::world::World world;
    robotik::RobotSession robot(world, dataFile("simple_revolute_robot.urdf"));
    robotik::CameraConfig config;
    config.frequency = 10.0;
    config.mount = downward();
    auto& camera = robot.sensors().add<robotik::Camera>("camera", config);
    EXPECT_TRUE(robot.resources().available("camera"));

    robot.step(Seconds(0.01));
    EXPECT_EQ(camera.samples(), 0u) << "no source, no frame";

    FlatSource source;
    camera.source(&source);
    int frames = 0;
    camera.onFrame([&frames](robotik::CameraFrame const&) noexcept { ++frames; });
    for (int i = 0; i < 100; ++i)
    {
        robot.step(Seconds(0.01));
    }
    EXPECT_NEAR(source.captures, 10, 1);
    EXPECT_EQ(frames, source.captures);
    EXPECT_EQ(camera.frame().rgb.width(), config.intrinsics.width);

    robot.resources().fail("camera");
    int const before = source.captures;
    for (int i = 0; i < 50; ++i)
    {
        robot.step(Seconds(0.01));
    }
    EXPECT_EQ(source.captures, before) << "a failed camera does not capture";
}

TEST(WorldModel, MonocularDetectionLandsOnTheTopPlane)
{
    robotik::CameraFrame frame;
    frame.pose = downward();
    frame.intrinsics = robotik::CameraIntrinsics::fromFov(
        320u, 240u, Radians(70.0 * std::numbers::pi / 180.0));
    frame.stamp = Seconds(1.0);

    // Top center of a cube at (0.3, 0.1), 4 cm high, seen by the camera.
    robotik::Vector3 const top{ 0.3, 0.1, 0.04 };
    auto const pixel = frame.intrinsics.project(frame.pose.inverse() * top);
    ASSERT_TRUE(pixel);

    robotik::PerceptionPipeline pipeline;
    pipeline.add<FixedDetector>(std::array<float, 2>{ static_cast<float>((*pixel)[0]),
                                                      static_cast<float>((*pixel)[1]) });
    robotik::WorldModel beliefs;
    beliefs.add("cube", { 0.25, 0.15, 0.02 }, { 0.04, 0.04, 0.04 });
    EXPECT_FALSE(beliefs.find("cube")->observed());

    beliefs.update(pipeline.process(frame));
    robotik::WorldObject const* cube = beliefs.find("cube");
    ASSERT_NE(cube, nullptr);
    EXPECT_TRUE(cube->observed());
    EXPECT_NEAR(cube->position.x, 0.3, 1e-3);
    EXPECT_NEAR(cube->position.y, 0.1, 1e-3);
    EXPECT_NEAR(cube->position.z, 0.02, 1e-9);
    EXPECT_EQ(cube->seen, Seconds(1.0));
}

TEST(WorldModel, GateRejectsOutliers)
{
    robotik::WorldModel beliefs;
    beliefs.gate(0.1);
    beliefs.add("cube", { 0.4, 0.2, 0.02 }, { 0.04, 0.04, 0.04 });
    EXPECT_TRUE(beliefs.observe("cube", { 0.43, 0.18, 0.5 }, 1.0f, Seconds(1.0)));
    EXPECT_NEAR(beliefs.find("cube")->position.z, 0.02, 1e-9);
    EXPECT_FALSE(beliefs.observe("cube", { 0.0, 0.0, 0.02 }, 1.0f, Seconds(2.0)));
    EXPECT_NEAR(beliefs.find("cube")->position.x, 0.43, 1e-9);
    EXPECT_EQ(beliefs.find("cube")->observations, 1u);
    EXPECT_FALSE(beliefs.observe("unknown", {}, 1.0f, Seconds(2.0)));
}

TEST(Localization, RecoversTheRobotPoseFromALandmark)
{
    robotik::Pose const robot{ { 1.0, -0.5, 0.0 }, robotik::rpy(0.0, 0.0, 0.4) };
    robotik::Pose const camera{ { 0.1, 0.0, 0.2 }, robotik::rpy(-2.0, 0.0, -1.57) };
    robotik::Landmark const tag{ 3, { { 1.4, -0.3, 0.0 }, robotik::Quaternion{} } };

    robotik::Detections detections;
    detections.camera = camera;
    robotik::Detection detection;
    detection.label = "tag";
    detection.id = 3;
    detection.pose = (robot * camera).inverse() * tag.pose;
    detections.items.push_back(detection);

    auto const found = robotik::localize(detections, std::span(&tag, 1u));
    ASSERT_TRUE(found);
    EXPECT_EQ(found->landmarks, 1u);
    EXPECT_NEAR(robotik::norm(found->pose.position - robot.position), 0.0, 1e-9);
    EXPECT_NEAR(robotik::yawOf(found->pose.rotation), 0.4, 1e-9);
}
